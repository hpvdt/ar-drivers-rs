@ms_precision_touchpad.rs should be revised to have the following features:

- instead of writing both raw and parsed message into multiple log files, the code should only write raw message into "examples/log/raw/all.log", each message should be written in one line
- move all code related to parsing of packets into a function "print_parsed", which takes a line of raw log as input, and return the parsed result as a structure
- when you are finished, run it and prompt for my input
- inspect the log file to ensure that the problem is bug-free