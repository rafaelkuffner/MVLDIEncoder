#include "RVLEncoder.h"
#include <sstream>
#include <iostream>

RVLEncoder::RVLEncoder() {
}

RVLEncoder::~RVLEncoder() {
	CloseHandle(_outFile);
	delete _output;
}

bool RVLEncoder::InitEncoder(int width, int height, string outputPath) {
	
	std::wstring stempb = std::wstring(outputPath.begin(), outputPath.end());
	LPCWSTR swb = stempb.c_str();
	_outFile = CreateFileW(swb, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	_output = (byte*)malloc(sizeof(short)*width*height);
	return true;
}

void RVLEncoder::EncodeVLE(int value)
{
	do

	{
		int nibble = value & 0x7; // lower 3 bits
		if (value >>= 3) nibble |= 0x8; // more to come
		_word <<= 4;
		_word |= nibble;
		if (++_nibblesWritten == 8) // output word
		{
			*_pBuffer++ = _word;
			_nibblesWritten = 0;
			_word = 0;
		}
	} while (value);
}

bool RVLEncoder::CompressRVL(short* input, int numPixels)
{
	_buffer = _pBuffer = (int*)_output;
	_nibblesWritten = 0;
	short *end = input + numPixels;
	short previous = 0;
	while (input != end)
	{
		int zeros = 0, nonzeros = 0;
		for (; (input != end) && !*input; input++, zeros++);
		EncodeVLE(zeros); // number of zeros
		for (short* p = input; (p != end) && *p++; nonzeros++);
		EncodeVLE(nonzeros); // number of nonzeros
		for (int i = 0; i < nonzeros; i++)
		{
			short current = *input++;
			int delta = current - previous;
			int positive = (delta << 1) ^ (delta >> 31);
			EncodeVLE(positive); // nonzero value
			previous = current;
		}
	}
	if (_nibblesWritten) // last few values
		*_pBuffer++ = _word << 4 * (8 - _nibblesWritten);
	int size = int((char*)_pBuffer - (char*)_buffer);
	DWORD written = 0;
	
	for (int i = 0; i < 4; i++)
		_sizeBuffer[3 - i] = (size >> (i * 8));

	WriteFile(_outFile, _sizeBuffer, 4, &written, NULL);
	WriteFile(_outFile, _output, size, &written, NULL);
	return true;
}

