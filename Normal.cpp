#include <utility> // std::pair
#include <list>
#include <fstream>
#include <CGAL/property_map.h>      
#include <CGAL/IO/read_points.h>           // 读取点云
#include <CGAL/IO/write_points.h>          // 保存点云
#include <CGAL/pca_estimate_normals.h>    // pca计算法向量
#include <CGAL/compute_average_spacing.h> // 计算点云平均密度
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Eigen_diagonalize_traits.h>
#include <chrono>
// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

// 定义存储点和法线的容器
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> PointVectorPair;


//// 重载输入运算符来读取逗号分隔的点坐标和向量
//std::istream& operator>>(std::istream& is, PointVectorPair& p) {
//	char comma;  // 用于读取逗号
//	double x, y, z, nx, ny, nz;
//	if (is >> x >> comma >> y >> comma >> z >> comma >> nx >> comma >> ny >> comma >> nz) {
//		p.first = Point(x, y, z);
//		p.second = Kernel::Vector_3(nx, ny, nz);
//	}
//	else {
//		is.setstate(std::ios_base::failbit);
//	}
//	return is;
//}

int main(int argc, char* argv[])
{
	auto start = std::chrono::high_resolution_clock::now();
	// 打开输入文件
	std::ifstream inputFile1("C:\\Users\\wyb\\Desktop\\PC\\pro\\G3.txt");
	// 打开输出文件
	std::ofstream outputFile1("output.txt");

	// 逐行读取文件
	std::string line1;
	while (std::getline(inputFile1, line1)) {
		// 替换逗号为空格
		size_t pos = 0;
		while ((pos = line1.find(",", pos)) != std::string::npos) {
			line1.replace(pos, 1, " ");
			pos += 1;
		}
		// 写入替换后的行到输出文件
		outputFile1 << line1 << std::endl;
	}

	// 关闭文件
	inputFile1.close();
	outputFile1.close();

	std::cout << "文件处理完成，逗号已替换为空格。" << std::endl;
	// 打开文本文件
	std::ifstream inputFile("E:/cppProjects/Normal/Normal/output.txt");
	if (!inputFile.is_open()) {
		std::cerr << "无法打开输入文件" << std::endl;
		return 1;
	}

	// 打开输出xyz文件
	std::ofstream outputFile("output.xyz");
	if (!outputFile.is_open()) {
		std::cerr << "无法打开输出文件" << std::endl;
		return 1;
	}

	std::string line;
	while (std::getline(inputFile, line)) {
		std::istringstream iss(line);
		double x, y, z;
		// 假设文本文件中的数据以空格分隔
		if (iss >> x >> y >> z) {
			// 写入到xyz文件中
			outputFile << x << " " << y << " " << z << std::endl;
		}
	}

	std::cout << "文件转换完成" << std::endl;

	// 关闭文件
	inputFile.close();
	outputFile.close();
	const std::string fname = CGAL::data_file_path("E:/cppProjects/Normal/Normal/output.xyz");
	std::cout << fname << std::endl;
	const char* output_filename = "G3.xyz";
	// -----------------------------------读取点云------------------------------------
	std::list<PointVectorPair> points;
	if (!CGAL::IO::read_points(fname, std::back_inserter(points),
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())))
	{
		std::cerr << "点云读取失败！！！ " << fname << std::endl;
		return -1;
	}
	bool knn_search = true;
	const int nb_neighbors = 18; // K近邻搜索的邻域点数
	std::cout << "Read Successful!" << std::endl;
	if (knn_search == true)
	{
		// ---------------------使用固定近邻点来计算法线-----------------------------
		///注意:pca_estimate_normals()需要一个范围的点以及属性映射来访问每个点的位置和法线。

		CGAL::pca_estimate_normals<CGAL::Parallel_if_available_tag>(points, nb_neighbors,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
			.normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
	}
	else
	{
		double spacing = CGAL::compute_average_spacing<CGAL::Parallel_if_available_tag>(points, nb_neighbors,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()));
		// ---------------------使用固定半径进行法线计算-----------------------------
		CGAL::pca_estimate_normals<CGAL::Parallel_if_available_tag>
			(points,
				0, // 当使用邻域半径时，K=0表示对返回的邻域数量没有限制
				CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
				.normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
				.neighbor_radius(2. * spacing)); // 使用平均间距的2倍作为搜索半径

	}
	std::cout << "Calculate Successful!" << std::endl;
	// ---------------------------------结果保存--------------------------------------
	if (!CGAL::IO::write_XYZ(output_filename, points,
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
		.normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
		.stream_precision(4)))
		return -1;
	std::cout << "Save Successful!" << std::endl;

	// 结束时间点
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	// 输出运行时间
	std::cout << "Elapsed time: " << elapsed.count() << "s\n";
	return 0;
}


