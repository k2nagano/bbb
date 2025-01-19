./test "( videotestsrc pattern=snow ! video/x-raw,width=256,height=200,framerate=15/1 ! x264enc ! rtph264pay name=pay0 pt=96 )"
