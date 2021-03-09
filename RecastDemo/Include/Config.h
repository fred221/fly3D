#pragma once
#include <map>
#define CONFIG_RULE  (2)
using namespace std;

class Config
{
public:
	void loadConfig(const string& key, const string& value);
	void clear();
	bool isFull();
private:
	map<string, string> m_config;
};
