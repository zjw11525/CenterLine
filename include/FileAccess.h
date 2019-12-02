#ifndef _FILEACCESS_H_
#define _FILEACCESS_H_

#include <iostream>
#include <vector>
#include <string>
#include "Point.h"

using std::string;
using std::vector;

class FileAccess
{
public:
	FileAccess() {}
	~FileAccess() {}
	void OutputResult(vector<Point> result, string filename, int index);
	void OutputResult(vector<vector<Point>> result, string filename, int index);
	vector<Point> ReadBoundary(string filename, vector<vector<int>> &edge);

private:
	double AngleCalc(vector<Point> &P, int index);
	vector<string> Split(string str, string pattern);
};

#endif