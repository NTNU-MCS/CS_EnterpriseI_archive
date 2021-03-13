//
//  interp.hpp
//  test_proj
//
//  Created by Erlend Basso on 08/03/2021.
//

#ifndef interp_h
#define interp_h

#include <Eigen/Dense>

// M: M \times M matrix of data to interpolate from
template <unsigned int M>

class BilinearInterpolator
{
public:
    using VectorMd = Eigen::Matrix<double, M, 1>;
    using MatrixMd = Eigen::Matrix<double, M, M>;
    using Vector2d = Eigen::Vector2d;

    BilinearInterpolator() {}

    BilinearInterpolator(const MatrixMd &F, const VectorMd &breakpoints_x, const VectorMd &breakpoints_y)
        : F_{F},
          breakpoints_x_{breakpoints_x},
          breakpoints_y_{breakpoints_y}
    {
    }

    void init(const MatrixMd &F, const VectorMd &breakpoints_x, const VectorMd &breakpoints_y)
    {
        F_ = F;
        breakpoints_x_ = breakpoints_x;
        breakpoints_y_ = breakpoints_y;
    }

    double interp(double x, double y)
    {
        int ind_x = 0;
        int ind_y = 0;

        for (int i = 0; i < M - 1; i++)
        {
            if (x >= breakpoints_x_(i) && x <= breakpoints_x_(i + 1))
            {
                x1_ = breakpoints_x_(i);
                x2_ = breakpoints_x_(i + 1);
                ind_x = i;
                break;
            }
        }
        for (int j = 0; j < M - 1; j++)
        {
            if (y >= breakpoints_y_(j) && y <= breakpoints_y_(j + 1))
            {
                y1_ = breakpoints_y_(j);
                y2_ = breakpoints_y_(j + 1);
                ind_y = j;
                break;
            }
        }

        Vector2d delta_x{x2_ - x, x - x1_};
        Vector2d delta_y{y2_ - y, y - y1_};

        return 1.0 / ((x2_ - x1_) * (y2_ - y1_)) * delta_x.transpose() * F_.block(ind_x, ind_y, 2, 2) * delta_y;
    }

private:
    MatrixMd F_;
    VectorMd breakpoints_x_;
    VectorMd breakpoints_y_;
    double x1_, x2_, y1_, y2_;
};

#endif /* interp_h */
