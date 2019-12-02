#include "FileAccess.h"
#include "Calculate.h"
#include "Optimizer.h"

int main(int argc, char *argv[])
{
	FileAccess fileaccess;
	Calculate calculate;
	vector<vector<int>> edge;
	int index = -1;
	string boundry_path;
	bool Is_Snake_Route = false;
	bool Is_Alpha_Route = false;
	vector<Point> boundary_local;
	double edge_width;

	if (argc == 2)
	{
		boundry_path = argv[1];
		index = boundry_path.find_last_of('_');
		if (index == -1)
		{
			std::cout << "Invalid file name format" << std::endl;
			return -1;
		}

		calculate.boundary = fileaccess.ReadBoundary(boundry_path, edge);
	}
	else
	{
		std::cout << "Usage: ./lineExt boundary_x.txt!" << std::endl;
		return -1;
	}
	std::cout << std::endl
			  << "------------------------------------------------------------------------>" << std::endl
			  << "Read: boundary" << boundry_path.substr(index) << " success!" << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////////中心线计算
	if (edge.size() == 0) //如果boundary文件不包含起始终止边则计算
	{
		edge = calculate.GetEdge(Is_Snake_Route, Is_Alpha_Route, edge_width);
	}

	if (edge.size() == 1) //弧形边
	{
		edge.clear();
		edge = calculate.GetEdge();
	}

	if (edge.size() == 2) //没有分叉
	{
		if (Is_Snake_Route == false)
		{
			Optimizer optimizer;
			optimizer.BoundaryOptimize(calculate.boundary, edge);
		}
		boundary_local = calculate.boundary;

		vector<Point> result = calculate.calculte(edge); //计算中心线
		std::cout << "calculate complete!" << std::endl;

		//优化点数
		Optimizer optimizer = Optimizer(result);
		result = optimizer.PointOptimize(calculate.boundary, edge, Is_Snake_Route);

		std::cout << "optimize complete and output result!" << std::endl;
		fileaccess.OutputResult(result, boundry_path, index);
	}
	else if (edge.size() > 2) //有分叉
	{
		boundary_local = calculate.boundary;
		vector<vector<Point>> result_multi_road = calculate.MultiRoad_calculte(edge, Is_Alpha_Route); //计算多条中心线

		//多叉路优化
		Optimizer optimizer;
		optimizer.MultiRoadOptimizer(result_multi_road);
		if (Is_Alpha_Route)
		{
			optimizer.RemoveOddPoint(boundary_local, result_multi_road, edge_width);
		}

		std::cout << "optimize complete and output result!" << std::endl;
		fileaccess.OutputResult(result_multi_road, boundry_path, index);
	}
	return 0;
}
