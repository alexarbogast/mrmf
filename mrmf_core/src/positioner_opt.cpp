#include <mrmf_core/positioner_opt.h>

namespace mrmf_core
{
double positionerOptDist2D(EigenSTL::vector_Vector3d& interest_points,
                           EigenSTL::vector_Vector3d& base_positions, 
                           double initial_guess)
{
    // create objective function
    auto objective = [interest_points, base_positions](double theta)
    {
        double cost = 0.0;

        Eigen::Rotation2Dd rot2(theta);
        for (int i = 0; i < interest_points.size(); i++)
        {
            Eigen::Vector2d pt2 = interest_points[i].head<2>();
            cost += ((rot2 * pt2) - base_positions[i].head<2>()).norm();
        }
        return cost;
    };

    auto gradient = [objective](double x_old, double step)
    {
        double x_new = x_old + step;  
        return (objective(x_new) - objective(x_old)) / step;
    };

    // gradient descent
    static unsigned int max_iter = 1000;
    static double tol = 1e-5, step_size = 0.1, gradient_step = 0.00001;

    unsigned int iter_count = 0;
    double gradientMagnitude = 1.0;

    double xk = initial_guess;
    while ((iter_count < max_iter) && (gradientMagnitude > tol))
    {
        double gr = gradient(xk, gradient_step);
        gradientMagnitude = sqrt(gr * gr);
        xk += -(gr * step_size);

        iter_count++;
    }

    return xk;
}

} // namespace mrmf_core