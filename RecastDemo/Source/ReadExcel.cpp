#include "ReadExcel.h"
#include <stdio.h>
#include <string>
vector<Config> gConfig;

void loadExcelFile(const char* filePath)
{
	FILE* fp = fopen(filePath, "rb");
	if (!fp)
	{
		return;
	}
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return;
	}
	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return;
	}
	char* buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return;
	}

	size_t readLen = fread(buf, bufSize, 1, fp);

	fclose(fp);
	if (readLen != 1)
	{
		delete[] buf;
		return;
	}  
	const static long INVALID = -1;
	long from_index = INVALID, end_index = INVALID;

	vector<string> json_vec;
	
	for (long index = 0; index < bufSize; ++index)
	{
		if (buf[index] == ',')
		{
			continue;
		}
		bool char_rule = checkSpecialChar(buf[index]);
		if (char_rule && from_index == INVALID)
		{
			from_index = index;
		}
		if (checkSplit(buf[index]) || checkString(buf[index]))
		{
			if (from_index != INVALID && from_index != index)
			{
				end_index = index;
			}
		}
		if (from_index == INVALID || end_index == INVALID)
		{
			continue;
		}
		filterStr(buf, from_index, end_index);
		json_vec.push_back(string(buf + from_index, buf + end_index + 1));
		from_index = INVALID;
		end_index  = INVALID;
	}
	gConfig.clear();
	Config config;
	for (long index = 0; index < json_vec.size() / 2; ++index)
	{
		config.loadConfig(json_vec[index * 2], json_vec[index * 2 + 1]);
		if (config.isFull())
		{
			gConfig.push_back(config);
			config.clear();
		}
	}
	delete[] buf;
	buf = nullptr;
}
bool checkSpecialChar(char c)
{
	switch (c)
	{
		case '{':
		case '}':
		case '[':
		case ']':
		case ':':
		return false;
	}
	return true;
}

bool checkString(char c)
{
	if (c == '\"') return true;
	return false;
}

bool checkSplit(char c)
{
	if (c == ':') return true;
	return false;
}

void filterStr(char* buf,long& from_index, long& end_index)
{
	if (checkString(buf[from_index])) from_index += 1;
	if (checkString(buf[end_index])) end_index -= 1;
}
