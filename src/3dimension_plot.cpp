#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

// 楕円球の最小二乗法フィッティング
std::vector<double> fittingEllipse(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
    Eigen::MatrixXd m(3, x.size());
    m << x, y, z;

    m.rowwise() -= m.colwise().mean(); // 平均を引く

    Eigen::MatrixXd mt = m.transpose();
    Eigen::MatrixXd M = m * mt;
    Eigen::MatrixXd Minv = M.inverse();
    Eigen::VectorXd msum = m.rowwise().sum();

    Eigen::VectorXd params = -Minv * msum;

    double a11 = params(0);
    double a22 = params(1);
    double a33 = params(2);
    double b1 = params(3);
    double b2 = params(4);
    double b3 = params(5);

    Eigen::VectorXd center = {b1 / (2 * a11), b2 / (2 * a22), b3 / (2 * a33)};
    double offset = -((b1 * b1) / (4 * a11) + (b2 * b2) / (4 * a22) + (b3 * b3) / (4 * a33) + 1);

    double a = sqrt(-offset / a11);
    double b = sqrt(-offset / a22);
    double c = sqrt(-offset / a33);

    return {a, b, c, center(0), center(1), center(2)};
}

int main() {
    // CSVファイルのパス
    std::string csvFilePath = "/home/sskr3/catkin_ws/src/scanner/data_csv/for_calibraiton/imu_data_2023-11-20_20-38.csv";

    // CSVファイルを読み込む
    std::ifstream csvFile(csvFilePath);
    std::vector<double> accelX, accelY, accelZ;
    std::string line;
    std::getline(csvFile, line); // ヘッダー行を読み飛ばす
    while (std::getline(csvFile, line)) {
        std::istringstream iss(line);
        double x, y, z;
        char comma;
        iss >> x >> comma >> y >> comma >> z;
        accelX.push_back(x);
        accelY.push_back(y);
        accelZ.push_back(z);
    }

    std::vector<double> params = fittingEllipse(accelX, accelY, accelZ);

    // データ情報を表示
    std::cout << "- * - data infomation - * -" << std::endl;
    std::cout << "acceleration_x mean: " << std::accumulate(accelX.begin(), accelX.end(), 0.0) / accelX.size() << std::endl;
    std::cout << "acceleration_y mean: " << std::accumulate(accelY.begin(), accelY.end(), 0.0) / accelY.size() << std::endl;
    std::cout << "acceleration_z mean: " << std::accumulate(accelZ.begin(), accelZ.end(), 0.0) / accelZ.size() << std::endl;
    std::cout << "a: " << params[0] << ", b: " << params[1] << ", c: " << params[2] << std::endl;
    std::cout << "center: " << params[3] << ", " << params[4] << ", " << params[5] << std::endl;
    std::cout << "- * " << std::endl;

    // 3Dプロット
    plt::figure();
    plt::scatter(accelX, accelY, accelZ, "black");
    plt::title("3D Acceleration Plot");
    plt::xlabel("x");
    plt::ylabel("y");
    plt::zlabel("z");

    // 楕円
    std::vector<double> xyX = params[0] * plt::linspace(0, 2 * M_PI, 129) + params[3];
    std::vector<double> xyY = params[1] * plt::sin(plt::linspace(0, 2 * M_PI, 129)) + params[4];
    plt::plot(xyX, xyY, "r");

    std::vector<double> yzY = params[1] * plt::cos(plt::linspace(0, 2 * M_PI, 129)) + params[4];
    std::vector<double> yzZ = params[2] * plt::sin(plt::linspace(0, 2 * M_PI, 129)) + params[5];
    plt::plot(yzY, yzZ, "b");

    std::vector<double> zxX = params[0] * plt::sin(plt::linspace(0, 2 * M_PI, 129)) + params[3];
    std::vector<double> zxZ = params[2] * plt::cos(plt::linspace(0, 2 * M_PI, 129)) + params[5];
    plt::plot(zxX, zxZ, "g");

    plt::show();

    return 0;
}
