#include "FileAccess.h"
#include <fstream>
#include <cassert>
#include <iomanip>
#include <cmath>

void FileAccess::OutputResult(vector<Point> result, string filename, int index)
{
	string dir = ".";
	string FileName = dir + "/centerline" + filename.substr(index);
	std::ofstream outfile(FileName);
	outfile.setf(std::ios::fixed);
	for (Point p : result)
	{
		// outfile << std::setprecision(2) << p.id << "," << p.x << " " << p.y << std::endl;
		outfile << std::setprecision(2) << p.x << " " << p.y << std::endl;
	}
	outfile.close();
}

void FileAccess::OutputResult(vector<vector<Point>> result, string filename, int index)
{
	string dir = ".";
	string FileName = dir + "/centerline" + filename.substr(index);
	std::ofstream outfile(FileName);
	outfile.setf(std::ios::fixed);

	for (int i = 0; i < result.size(); i++)
	{
		for (Point p : result[i])
		{
			//outfile << std::setprecision(2) << p.id << "," << p.x << " " << p.y << std::endl;
			outfile << std::setprecision(2) << p.x << " " << p.y << std::endl;
		}
		outfile << std::endl;
	}

	outfile.close();
}

vector<Point> FileAccess::ReadBoundary(string filename, vector<vector<int>> &edge)
{
	vector<Point> boundary;
	std::ifstream infile;
	infile.open(filename.data());
	assert(infile.is_open());

	string s;
	while (getline(infile, s))
	{
		//cout << s << endl;
		vector<string> substr = this->Split(s, " ");
		if (substr[0] == "V")
		{
			int id = stoi(substr[1]);
			double x = stod(substr[2]);
			double y = stod(substr[3]);
			boundary.push_back(Point(id, x, y));
		}
		else if (substr[0] == "E1" || substr[0] == "E2")
		{
			int x = stod(substr[1]);
			int y = stod(substr[2]);
			edge.push_back({x, y});
		}
		// else
		// {
		// 	std::cout << "boundary file error!" << std::endl;
		// }
	}

	//id重排列
	// for (int i = 0; i < boundary.size(); i++)
	// {
	// 	boundary[i].id = i;
	// }

	for (int i = 0; i < boundary.size(); i++)
	{
		boundary[i].angle = this->AngleCalc(boundary, i);
	}

	infile.close();

	return boundary;
}

double FileAccess::AngleCalc(vector<Point> &p, int index)
{
	int n = p.size();
	Point from = p[(index + n - 1) % n];
	Point cur = p[index];
	Point to = p[(index + 1) % n];
	double x1 = cur.x - from.x;
	double y1 = cur.y - from.y;
	double x2 = to.x - cur.x;
	double y2 = to.y - cur.y;
	double ang = (x1 * x2 + y1 * y2) / (sqrt((x1 * x1 + y1 * y1) * (x2 * x2 + y2 * y2)));

	//std::cout << std::acos(ang) * (180 / 3.1415) << std::endl;
	return std::acos(ang) * (180 / 3.1415);
}

vector<string> FileAccess::Split(string str, string pattern)
{
	string::size_type pos;
	vector<string> result;

	str += pattern;
	int size = str.size();

	for (int i = 0; i < size; i++)
	{
		pos = str.find(pattern, i);
		if (pos < size)
		{
			std::string s = str.substr(i, pos - i);
			result.push_back(s);
			i = pos + pattern.size() - 1;
		}
	}
	return result;
}