./test "( videotestsrc pattern=ball ! video/x-raw,width=256,height=400,framerate=15/1 ! x264enc ! rtph264pay name=pay0 pt=96 )"
