#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <random>

class KalmanFilter {
  const int dimU, dimX, dimZ;
  Eigen::MatrixXd A, B, H, Q, R;
  Eigen::MatrixXd P, K;
  Eigen::MatrixXd x_opt;
  const Eigen::MatrixXd Idt;
public:
  KalmanFilter(Eigen::MatrixXd mA, Eigen::MatrixXd mB, Eigen::MatrixXd mH,
               Eigen::MatrixXd mQ, Eigen::MatrixXd mR);
  // 默认控制量为 0
  Eigen::MatrixXd iter(const Eigen::VectorXd z);
  // z: 观测值; u: 控制量
  Eigen::MatrixXd iter(const Eigen::VectorXd z, const Eigen::VectorXd u);
};

KalmanFilter::KalmanFilter(Eigen::MatrixXd mA, Eigen::MatrixXd mB,
                           Eigen::MatrixXd mH, Eigen::MatrixXd mQ,
                           Eigen::MatrixXd mR)
    : dimU(mB.rows()), dimX(mA.rows()), dimZ(mH.rows()), A(mA), B(mB), H(mH),
      Q(mQ), R(mR), Idt(Eigen::MatrixXd::Identity(dimX, dimX)) {
  // 初始化最优估计值 (初始状态)
  x_opt = Eigen::VectorXd(dimX);
  // 初始化先验估计协方差 (初始协方差)
  P = Eigen::MatrixXd::Identity(dimX, dimX);
  // 初始化卡尔曼增益
  K = Eigen::MatrixXd(dimX, dimZ);
}

Eigen::MatrixXd KalmanFilter::iter(const Eigen::VectorXd z) {
  Eigen::VectorXd u(dimU);
  iter(z,u);
  return x_opt;
}

Eigen::MatrixXd KalmanFilter::iter(const Eigen::VectorXd z,
                                   const Eigen::VectorXd u) {
  // 1. 状态转移方程
  Eigen::VectorXd x_est(dimX);
  x_est = A*x_opt + B*u;
  // 2. 观测方程
  // z = H*x_mes + v
  // 3. 先验估计 x_est 的协方差
  P = A*P*A.transpose() + Q;
  // 4. 更新卡尔曼增益 K
  K = P*H.transpose() * (H*P*H.transpose()+R).inverse();
  // 5. 修正估计
  x_opt = x_est + K*(z-H*x_est);
  // 6. 后验估计协方差
  P = (Idt - K*H) * P;
  return x_opt;
}

int main() {
  // 产生随机种子, 每一个seed()会产生一个随机数
  std::random_device seed;
  // 随机数生成器
  std::default_random_engine gen(seed());
  // 带有随机误差的观测值序列
  std::vector<double> data;
  for (int i=0; i<100; ++i) {
    std::normal_distribution<double> dis(0,10);
    // xk = 10k + v; v~N(0,10)
    data.emplace_back(10*i + dis(gen));
  }

  // 初始化卡尔曼滤波器参数
  Eigen::MatrixXd A(2,2),B(2,2),H(1,2),Q(2,2),R(1,1);
  A << 1, 1, 0, 1;
  H << 1, 0;
  Q << 0.5, 0, 0, 0.5;
  R << 100;
  Eigen::VectorXd x(2), z(1), u(2);
  KalmanFilter kalman(A,B,H,Q,R);

  // 开始卡尔曼滤波
  for (int i=0; i<data.size(); ++i) {
    z << data[i];
    Eigen::VectorXd tmp = kalman.iter(z,u);
    std::cout << tmp.transpose() << std::endl;
  }

  return 0;
}

