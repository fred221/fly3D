#include "Config.h"
#include <vector>

using namespace std;

void loadExcelFile(const char* filePath);
void filterStr(char* buf, long& from_index, long& end_index);
bool checkSpecialChar(char c);
bool checkString(char c);
bool checkSplit(char c);
extern vector<Config> gConfig;