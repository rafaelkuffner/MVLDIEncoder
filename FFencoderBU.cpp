#include "FFencoder.h"


FFencoder::FFencoder(std::string fname,int width, int height)
{
	
	c = NULL;
	uint8_t endcode[] = { 0, 0, 1, 0xb7 };

	filename = fname.c_str();

	avcodec_register_all();	
	codec = avcodec_find_encoder(AV_CODEC_ID_H264);

	if (!codec) {
		fprintf(stderr, "Codec not found\n");
		exit(1);
	}
	c = avcodec_alloc_context3(codec);
	if (!c) {
		fprintf(stderr, "Could not allocate video codec context\n");
		exit(1);
	}

	pkt = av_packet_alloc();
	if (!pkt)
		exit(1);


	c->width = width;
	c->height = height;
	/* frames per second */
	c->time_base.num = 1;
	c->time_base.den = 30;

	c->framerate.num =  30;
	c->framerate.den = 1;
	c->pix_fmt = AV_PIX_FMT_YUV444P;

	if (codec->id == AV_CODEC_ID_H264){
		av_opt_set(c->priv_data, "preset", "veryslow", 0);
		av_opt_set(c->priv_data, "crf", "0", 0);
	}


	/* emit one intra frame every ten frames
	* check frame pict_type before passing frame
	* to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
	* then gop_size is ignored and the output of encoder
	* will always be I frame irrespective to gop_size
	*/
	//c->gop_size = 10;
	//c->max_b_frames = 1;
	/* put sample parameters */
	//c->bit_rate = 400000;
	/* resolution must be a multiple of two */



	/* open it */
	if (avcodec_open2(c, codec, NULL) < 0) {
		fprintf(stderr, "Could not open codec\n");
		exit(1);
	}

	f = fopen(filename, "wb");
	if (!f) {
		fprintf(stderr, "Could not open %s\n", filename);
		exit(1);
	}

	frame = av_frame_alloc();
	if (!frame) {
		fprintf(stderr, "Could not allocate video frame\n");
		exit(1);
	}
	frame->format = c->pix_fmt;
	frame->width = c->width;
	frame->height = c->height;

	int ret = av_frame_get_buffer(frame, 32);
	if (ret < 0) {
		fprintf(stderr, "Could not allocate the video frame data\n");
		exit(1);
	}

}


FFencoder::~FFencoder()
{
}

void encode(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt,
	FILE *outfile)
{
	int ret;

	/* send the frame to the encoder */
	ret = avcodec_send_frame(enc_ctx, frame);
	if (ret < 0) {
		fprintf(stderr, "Error sending a frame for encoding\n");
		exit(1);
	}

	while (ret >= 0) {
		ret = avcodec_receive_packet(enc_ctx, pkt);
		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
			return;
		else if (ret < 0) {
			fprintf(stderr, "Error during encoding\n");
			exit(1);
		}

		printf("Write frame %llu (size=%5d)\n", pkt->pts, pkt->size);
		fwrite(pkt->data, 1, pkt->size, outfile);
		av_packet_unref(pkt);
	}
}

void FFencoder::processFrame(int i, uint8_t* data){
	fflush(stdout);

	SwsContext * ctx = sws_getContext(c->width, c->height,
		AV_PIX_FMT_RGBA,
		c->width, c->height,
		AV_PIX_FMT_YUV444P,
		0, 0, 0, 0);

	uint8_t *inData[1] = { data };
	int      inLinesize[1] = { 4 * c->width };
	sws_scale(ctx, inData, inLinesize, 0, c->height,
		frame->data, frame->linesize);
	/* make sure the frame data is writable */
	int ret = av_frame_make_writable(frame);
	if (ret < 0)
		exit(1);
	
	frame->pts = i;
	/* encode the image */
	encode(c, frame, pkt, f);
}

void FFencoder::finalizeEncoder(){
	encode(c, NULL, pkt, f);

	/* add sequence end code to have a real MPEG file */
	uint8_t endcode[] = { 0, 0, 1, 0xb7 };
	fwrite(endcode, 1, sizeof(endcode), f);
	fclose(f);

	avcodec_free_context(&c);
	av_frame_free(&frame);
	av_packet_free(&pkt);

}