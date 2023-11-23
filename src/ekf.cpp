#include <iostream>
#include <Eigen/Dense>
#include <cmath>

class ExtKalmanFilter {
public:
    ExtKalmanFilter(double period_ms) : dts(period_ms / 1000.0) {
        // Initialize angular velocity
        gyro_angular_vel << std::sin(deg2rad(10.0)), std::sin(deg2rad(10.0)), std::sin(deg2rad(0.0));

        // Initialize observation matrix H
        H = Eigen::MatrixXd::Identity(2, 2);

        // Initialize system noise variance Q
        Q = Eigen::MatrixXd::Identity(2, 2);
        Q *= 1.74E-2 * dts * dts;

        // Initialize observation noise variance R
        R = Eigen::MatrixXd::Identity(2, 2);
        R *= 1.0 * dts * dts;

        // Initialize true status
        x_true << 0.0, 0.0;

        // Initialize prediction
        x_bar = x_true;

        // Initialize estimation
        x_hat = x_true;

        // Initialize covariance
        P = Q;

        // Initialize jacobian matrix of H
        jacobianH = Eigen::MatrixXd::Identity(2, 2);
    }

    Eigen::VectorXd getExtKalman() {
        // Ground Truth
        Eigen::MatrixXd tri(2, 3);
        tri << std::cos(x_true(0)), std::sin(x_true(0)), std::tan(x_true(0)),
               std::cos(x_true(1)), std::sin(x_true(1)), std::tan(x_true(1));

        Eigen::MatrixXd Q(2, 3);
        Q << 1, tri(0, 1) * tri(1, 2), tri(0, 0) * tri(1, 2),
             0, tri(0, 0), -tri(0, 1);

        x_true = x_true + Q * gyro_angular_vel * dts;

        // [step1] prediction
        tri << std::cos(x_hat(0)), std::sin(x_hat(0)), std::tan(x_hat(0)),
               std::cos(x_hat(1)), std::sin(x_hat(1)), std::tan(x_hat(1));
        Q << 1, tri(0, 1) * tri(1, 2), tri(0, 0) * tri(1, 2),
             0, tri(0, 0), -tri(0, 1);

        x_bar = x_hat + Q * gyro_angular_vel * dts;

        // jacobian matrix
        Eigen::VectorXd g = gyro_angular_vel;
        tri << std::cos(x_bar(0)), std::sin(x_bar(0)), std::tan(x_bar(0)),
               std::cos(x_bar(1)), std::sin(x_bar(1)), std::tan(x_bar(1));

        jacobianH << 1.0 + (tri(0, 0) * tri(1, 2) * g(1) - tri(0, 1) * tri(1, 2) * g(2)) * dts, (tri(0, 1) / tri(1, 0) / tri(1, 0) * g(1) + tri(0, 0) / tri(1, 0) / tri(1, 0) * g(2)) * dts,
                     -(tri(0, 1) * g(1) + tri(0, 0) * g(2)) * dts, 1.0;

        // pre_covariance
        Eigen::MatrixXd P_bar = jacobianF * P * jacobianF.transpose() + Q;

        // observation
        Eigen::VectorXd w = Eigen::VectorXd::Random(2) * std::sqrt(R(0, 0));
        Eigen::VectorXd y = H * x_true + w;

        // [step2] update the filter
        Eigen::MatrixXd S = H * P_bar * H.transpose() + R;
        Eigen::MatrixXd K = P_bar * H.transpose() * S.inverse();

        // estimation
        Eigen::VectorXd e = y - jacobianH * x_bar;
        x_hat = x_bar + K * e;

        // post_covariance
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_hat.size(), x_hat.size());
        P = (I - K * H) * P_bar;

        Eigen::VectorXd result(4);
        result << x_true, y, x_hat, P.diagonal();
        return result;
    }

private:
    double dts;  // Time step
    Eigen::VectorXd gyro_angular_vel;  // Angular velocity

    Eigen::MatrixXd H;  // Observation matrix
    Eigen::MatrixXd Q;  // System noise variance
    Eigen::MatrixXd R;  // Observation noise variance

    Eigen::VectorXd x_true;  // True status
    Eigen::VectorXd x_bar;   // Prediction
    Eigen::VectorXd x_hat;   // Estimation
    Eigen::MatrixXd P;       // Covariance

    Eigen::MatrixXd jacobianH;  // Jacobian matrix of H

    double deg2rad(double degree) {
        return degree * M_PI / 180.0;
    }
};

int main() {
    double period_ms = 100.0;
    int frame_cnt = static_cast<int>(12000.0 / period_ms);

    ExtKalmanFilter ekf(period_ms);

    for (int i = 0; i < frame_cnt; ++i) {
        Eigen::VectorXd result = ekf.getExtKalman();

        // Print the result (You can modify this part according to your needs)
        std::cout << "x_true: " << result.segment(0, 2).transpose() << std::endl;
        std::cout << "y: " << result.segment(2, 2).transpose() << std::endl;
        std::cout << "x_hat: " << result.segment(4, 2).transpose() << std::endl;
        std::cout << "P: " << result.segment(6, 2).transpose() << std::endl;
    }

    return 0;
}
