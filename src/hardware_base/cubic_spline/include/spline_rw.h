//
// Created by dinhnambkhn on 22/11/2024.
//

#ifndef SPLINE_RW_H
#define SPLINE_RW_H
#include <vector>
#include <Eigen/Dense>

class Spline {
public:
    // Fit the spline to given parameterization and values
    void fit(const std::vector<double> &t, const std::vector<double> &y) {
        n_ = t.size() - 1;
        t_ = t;

        Eigen::VectorXd h(n_);
        for (int i = 0; i < n_; ++i) {
            h(i) = t[i + 1] - t[i];
        }

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_ + 1, n_ + 1);
        Eigen::VectorXd R = Eigen::VectorXd::Zero(n_ + 1);

        // Natural boundary conditions
        A(0, 0) = 1;
        A(n_, n_) = 1;

        for (int i = 1; i < n_; ++i) {
            A(i, i - 1) = h(i - 1);
            A(i, i) = 2 * (h(i - 1) + h(i));
            A(i, i + 1) = h(i);
            R(i) = 6 * ((y[i + 1] - y[i]) / h(i) - (y[i] - y[i - 1]) / h(i - 1));
        }

        Eigen::VectorXd M = A.colPivHouseholderQr().solve(R);

        a_ = Eigen::VectorXd(n_);
        b_ = Eigen::VectorXd(n_);
        c_ = M.head(n_);
        d_ = Eigen::VectorXd(n_);

        for (int i = 0; i < n_; ++i) {
            a_(i) = y[i];
            b_(i) = (y[i + 1] - y[i]) / h(i) - h(i) * (2 * c_(i) + M(i + 1)) / 6;
            d_(i) = (M(i + 1) - c_(i)) / (6 * h(i));
        }
    }

    // Evaluate the spline at a given t
    double evaluate(const double t) const {
        const int i = findSegment(t);
        const double dt = t - t_[i];
        return a_(i) + b_(i) * dt + c_(i) * dt * dt / 2 + d_(i) * dt * dt * dt;
    }

    // Compute the first derivative of the spline at a given t
    double derivative(const double t) const {
        const int i = findSegment(t);
        const double dt = t - t_[i];
        return b_(i) + c_(i) * dt + 3 * d_(i) * dt * dt;
    }

private:
    Eigen::VectorXd a_, b_, c_, d_; // Spline coefficients
    std::vector<double> t_; // Parameterization points
    int n_ = 0; // Number of segments
    int findSegment(const double t) const {
        for (int i = 0; i < n_; ++i) {
            if (t >= t_[i] && t <= t_[i + 1]) {
                return i;
            }
        }
        return n_ - 1; // Last segment
    }
};

#endif //SPLINE_RW_H
