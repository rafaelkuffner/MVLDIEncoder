#pragma once

extern "C"{
	#include <libavcodec/avcodec.h>
	#include <libavutil/opt.h>
	#include <libavutil/imgutils.h>
	#include <libswscale/swscale.h>
}

#include <string>

class FFencoder
{
public:
	FFencoder(std::string fname,int width, int height);
	~FFencoder();
	void processFrame(int i, uint8_t* data);
	void finalizeEncoder();

private:
	const char *filename;
	const AVCodec *codec;
	AVCodecContext *c;
	FILE *f;
	AVFrame *frame;
	AVPacket *pkt;
	
};

