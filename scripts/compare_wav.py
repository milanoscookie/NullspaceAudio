#!/usr/bin/env python3

import argparse
import math
import wave
from pathlib import Path

import numpy as np

try:
    from pesq import pesq as pesq_score  # type: ignore[import-not-found]
except ImportError:
    pesq_score = None

try:
    from pystoi import stoi as stoi_score  # type: ignore[import-not-found]
except ImportError:
    stoi_score = None

try:
    from scipy.signal import resample_poly
except ImportError:
    resample_poly = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare two WAV files with perceptual and waveform metrics."
    )
    parser.add_argument("reference", type=Path, help="Reference WAV file")
    parser.add_argument("degraded", type=Path, help="Comparison WAV file")
    parser.add_argument(
        "--pesq-mode",
        choices=("wb", "nb"),
        default="wb",
        help="PESQ mode: wideband (16 kHz) or narrowband (8 kHz)",
    )
    return parser.parse_args()


def _decode_pcm(data: bytes, sample_width: int) -> np.ndarray:
    if sample_width == 1:
        samples = np.frombuffer(data, dtype=np.uint8).astype(np.float32)
        return (samples - 128.0) / 128.0
    if sample_width == 2:
        samples = np.frombuffer(data, dtype="<i2").astype(np.float32)
        return samples / 32768.0
    if sample_width == 3:
        raw = np.frombuffer(data, dtype=np.uint8).reshape(-1, 3)
        samples = (
            raw[:, 0].astype(np.int32)
            | (raw[:, 1].astype(np.int32) << 8)
            | (raw[:, 2].astype(np.int32) << 16)
        )
        sign_mask = 1 << 23
        samples = (samples ^ sign_mask) - sign_mask
        return samples.astype(np.float32) / 8388608.0
    if sample_width == 4:
        pcm = np.frombuffer(data, dtype="<i4").astype(np.float32)
        return pcm / 2147483648.0
    raise ValueError(f"Unsupported sample width: {sample_width} bytes")


def load_wav(path: Path) -> tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as wav_file:
        sample_rate = wav_file.getframerate()
        num_channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        frames = wav_file.readframes(wav_file.getnframes())

    samples = _decode_pcm(frames, sample_width)
    if num_channels > 1:
        samples = samples.reshape(-1, num_channels).mean(axis=1)
    return samples.astype(np.float32), sample_rate


def _resample_linear(signal: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    if src_rate == dst_rate or signal.size == 0:
        return signal.astype(np.float32, copy=False)
    duration = signal.size / float(src_rate)
    dst_len = max(1, int(round(duration * dst_rate)))
    src_x = np.linspace(0.0, duration, num=signal.size, endpoint=False)
    dst_x = np.linspace(0.0, duration, num=dst_len, endpoint=False)
    return np.interp(dst_x, src_x, signal).astype(np.float32)


def resample_signal(signal: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    if src_rate == dst_rate:
        return signal.astype(np.float32, copy=False)
    if resample_poly is not None:
        gcd = math.gcd(src_rate, dst_rate)
        up = dst_rate // gcd
        down = src_rate // gcd
        return resample_poly(signal, up, down).astype(np.float32)
    return _resample_linear(signal, src_rate, dst_rate)


def align_signals(
    reference: np.ndarray, degraded: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    min_len = min(reference.size, degraded.size)
    if min_len == 0:
        raise ValueError("One of the WAV files is empty")
    return reference[:min_len], degraded[:min_len]


def rms(signal: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(signal), dtype=np.float64)))


def snr_db(reference: np.ndarray, degraded: np.ndarray) -> float:
    noise = reference - degraded
    ref_power = np.mean(np.square(reference), dtype=np.float64)
    noise_power = np.mean(np.square(noise), dtype=np.float64)
    if noise_power <= 1e-20:
        return float("inf")
    if ref_power <= 1e-20:
        return float("-inf")
    return float(10.0 * np.log10(ref_power / noise_power))


def segmental_snr_db(
    reference: np.ndarray, degraded: np.ndarray, frame_len: int = 256
) -> float:
    frame_count = min(reference.size, degraded.size) // frame_len
    if frame_count == 0:
        return snr_db(reference, degraded)
    scores = []
    for frame_index in range(frame_count):
        start = frame_index * frame_len
        stop = start + frame_len
        ref_frame = reference[start:stop]
        deg_frame = degraded[start:stop]
        noise_frame = ref_frame - deg_frame
        ref_power = np.mean(np.square(ref_frame), dtype=np.float64)
        noise_power = np.mean(np.square(noise_frame), dtype=np.float64)
        if ref_power <= 1e-20 or noise_power <= 1e-20:
            continue
        score = 10.0 * np.log10(ref_power / noise_power)
        scores.append(float(np.clip(score, -10.0, 35.0)))
    if not scores:
        return float("inf")
    return float(np.mean(scores))


def normalized_correlation(reference: np.ndarray, degraded: np.ndarray) -> float:
    denominator = np.linalg.norm(reference) * np.linalg.norm(degraded)
    if denominator <= 1e-20:
        return 0.0
    return float(np.dot(reference, degraded) / denominator)


def try_pesq(
    reference: np.ndarray, degraded: np.ndarray, sample_rate: int, mode: str
) -> tuple[str, float | None]:
    if pesq_score is None:
        return "pesq", None
    target_rate = 16000 if mode == "wb" else 8000
    ref_resampled = resample_signal(reference, sample_rate, target_rate)
    deg_resampled = resample_signal(degraded, sample_rate, target_rate)
    ref_resampled, deg_resampled = align_signals(ref_resampled, deg_resampled)
    try:
        return "pesq", float(
            pesq_score(target_rate, ref_resampled, deg_resampled, mode)
        )
    except Exception:
        return "pesq", None


def try_stoi(
    reference: np.ndarray, degraded: np.ndarray, sample_rate: int
) -> tuple[str, float | None]:
    if stoi_score is None:
        return "stoi", None
    reference, degraded = align_signals(reference, degraded)
    try:
        return "stoi", float(
            stoi_score(reference, degraded, sample_rate, extended=False)
        )
    except Exception:
        return "stoi", None


def format_metric(name: str, value: float | None) -> str:
    if value is None:
        return f"{name}: unavailable"
    if math.isinf(value):
        return f"{name}: {value}"
    return f"{name}: {value:.4f}"


if __name__ == "__main__":
    args = parse_args()

    reference, reference_rate = load_wav(args.reference)
    degraded, degraded_rate = load_wav(args.degraded)

    target_rate = reference_rate
    if degraded_rate != target_rate:
        degraded = resample_signal(degraded, degraded_rate, target_rate)
    reference, degraded = align_signals(reference, degraded)

    sample_rate = target_rate

    metrics = [
        ("reference_sample_rate_hz", float(reference_rate)),
        ("degraded_sample_rate_hz", float(degraded_rate)),
        ("aligned_samples", float(reference.size)),
        ("reference_rms", rms(reference)),
        ("degraded_rms", rms(degraded)),
        ("mse", float(np.mean(np.square(reference - degraded), dtype=np.float64))),
        ("rmse", rms(reference - degraded)),
        ("snr_db", snr_db(reference, degraded)),
        ("segmental_snr_db", segmental_snr_db(reference, degraded)),
        ("normalized_correlation", normalized_correlation(reference, degraded)),
    ]

    pesq_name, pesq_value = try_pesq(reference, degraded, sample_rate, args.pesq_mode)
    stoi_name, stoi_value = try_stoi(reference, degraded, sample_rate)

    print(f"reference: {args.reference}")
    print(f"degraded:  {args.degraded}")
    print(f"analysis_sample_rate_hz: {sample_rate}")
    for name, value in metrics:
        print(format_metric(name, value))
    print(format_metric(pesq_name, pesq_value))
    print(format_metric(stoi_name, stoi_value))

    missing = []
    if pesq_value is None:
        missing.append("pesq")
    if stoi_value is None:
        missing.append("pystoi")
    if missing:
        print(
            "optional_dependencies_missing: "
            + ", ".join(missing)
            + " (install with `pip install "
            + " ".join(missing)
            + " scipy`)"
        )
