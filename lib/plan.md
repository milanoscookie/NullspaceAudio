# Initial Plan

- Need a clock to generate a signal
- From clock, do some math in order to generate a value. This uses values in a sensor config 
- Fill up input buffer
- Once input buffer is full, send over
- Processing todo by student
- Send back output buffer
- Has different options to write to file, print to standard out, or ignore
- Set expected latency


- Run input and output on seperate threads?
- If output isn't sent be latency + input sending time, we send zeros



# Try | 

### Input threads
- Take in audio as part of a callback, this is the out of ear micrphone
- propagates audio through the different paths to get the in ear microphone
- dynamically updates S and the noise profile
- once you have in ear and out of ear mics, you want to send the micBlock to the micQueue


### Output thread
- reads the value of controlBuf, propigates the it through the control path and adds it with the micBlcok from micQueue to get the output block
- if there is no control value published, set it to all zeros
- consumes the micBlock in micQueue


### DSP Thread
- Gets input block from micQueue
- does some processing in a function
- Produces the next value to be placed in the controlBuf


## Notes on timing:
- The output thread should systemLatency_ seconds after the input thread gets the audio block. This means that the DSP thread should produce the control value within systemLatency_ seconds of the input thread getting the audio block. If it doesn't, then the output thread should use zeros for the control value.
