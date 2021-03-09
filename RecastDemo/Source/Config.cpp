#include "Config.h"
const string configKey[CONFIG_RULE] = { "asset","enterPoint" };
void Config::loadConfig(const string& key, const string& value)
{
	if (isFull())
	{
		return;
	}
	bool is_rule = false;
	for (size_t index = 0; index < CONFIG_RULE; ++index)
	{
		if (key == configKey[index]) is_rule = true;
	}
	if (!is_rule)
	{
		return;
	}
	m_config.insert(make_pair(key, value));
}

void Config::clear()
{
	m_config.clear();
}

bool Config::isFull()
{
	if (m_config.size() >= CONFIG_RULE)
	{
		return true;
	}
	return false;
}
