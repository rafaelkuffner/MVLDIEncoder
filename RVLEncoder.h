#pragma once

#include <Windows.h>
#include <string>
using namespace std;

class RVLEncoder
{
private:
	int *_buffer, *_pBuffer, _word, _nibblesWritten;
	byte *_output;
	HANDLE _outFile;
	byte _sizeBuffer[4];

public:
	RVLEncoder();
	virtual ~RVLEncoder();

private:
	void EncodeVLE(int value);

public:
	bool	InitEncoder(int width, int height, string outputPath);
	bool	CompressRVL(short* input, int numPixels);

};