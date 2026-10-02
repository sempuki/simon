# JSBSim tools

`convert.py` reads aircraft in the format of
[JSBSim](https://github.com/JSBSim-Team/jsbsim), an open-source flight dynamics
model (LGPL 2.1 or later), and writes them in simon's aircraft format. JSBSim's
aircraft and engine files have their own authors and licenses. The converter
copies them into each converted file's header.
