#ifndef KF_HPP
#define KF_HPP
#include <Eigen/Dense>

class Kalman3
{
public:
    using Vec3  = Eigen::Matrix<double,3,1>;
    using Mat3  = Eigen::Matrix<double,3,3>;
    bool kf_start = false;

    Kalman3()
    {
        // default: identity dynamics & measurement
        F_.setIdentity();
        H_.setIdentity();

        x_.setZero();
        P_.setIdentity() * 1e3;   // high uncertainty by default

        Q_.setIdentity() * 1e-2;  // small process noise
        R_.setIdentity() * 1e-1;  // measurement noise
    }

    // Initialize with first measurement and covariance (optional)
    void init(const Vec3 &initial_pos, const Mat3 &initial_P = Mat3::Identity() * 1.0)
    {
        x_ = initial_pos;
        P_ = initial_P;
        kf_start = true;
    }

    // Set process noise covariance (3x3)
    void setProcessNoise(const Mat3 &Q) { Q_ = Q; }

    // Set measurement noise covariance (3x3)
    void setMeasurementNoise(const Mat3 &R) { R_ = R; }

    // Predict step (no control input)
    void predict()
    {
        x_ = F_ * x_;               // with F=I this is a no-op except for numerical effects
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    // Update step with measurement z (3x1)
    void update(const Vec3 &z)
    {
        Vec3 y = z - H_ * x_;                           // innovation
        Mat3 S = H_ * P_ * H_.transpose() + R_;         // innovation covariance
        Mat3 K = P_ * H_.transpose() * S.inverse();     // kalman gain

        x_ = x_ + K * y;
        Mat3 I = Mat3::Identity();
        P_ = (I - K * H_) * P_;
    }

    // convenience: predict then update
    void step(const Vec3 &z)
    {
        predict();
        update(z);
    }

    Vec3 state() const { return x_; }
    Mat3 cov()   const { return P_; }

    // optional setters for F/H if you want to change model later
    void setF(const Mat3 &F) { F_ = F; }
    void setH(const Mat3 &H) { H_ = H; }

private:
    // state
    Vec3 x_;
    Mat3 P_;

    // model matrices
    Mat3 F_;
    Mat3 H_;

    // noise covariances
    Mat3 Q_;
    Mat3 R_;
};

#endif