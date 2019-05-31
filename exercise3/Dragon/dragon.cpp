#include "io.h"
#include "points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
    RegistrationCostFunction(const Point2D& _p, const Point2D& _q, const Weight& _wt): p(_p), q(_q), wt(_wt){}

    template<typename T>
    bool operator()(const T* const deg, const T* const tx, const T* const ty, T* residual) const{
        Eigen::Matrix<T,2,2> rotation;
        T angle_rad = deg[0] * T(0.0174533);
        rotation(0,0) = cos(angle_rad);
        rotation(0,1) = -sin(angle_rad);
        rotation(1,0) = sin(angle_rad);
        rotation(1,1) = cos(angle_rad);

        Eigen::Matrix<T,2,1> translation(tx[0],ty[0]);
        Eigen::Matrix<T,2,1> point_p(T(p.x),T(p.y));
        Eigen::Matrix<T,2,1> point_q(T(q.x),T(q.y));

        Eigen::Matrix<T,2,1> translated_p = (rotation * point_p) + translation;
        T error = (translated_p - point_q).norm();

        residual[0] = T(wt.w) * error;

        return true;
    }

private:
    const Point2D p;
    const Point2D q;
    const Weight wt;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const std::string file_path_weights = "../data/weights_dragon.txt";

	const auto points_p = read_points_from_file<Point2D>(file_path_1);
	const auto points_q = read_points_from_file<Point2D>(file_path_2);
    const auto weights = read_points_from_file<Weight>(file_path_weights);

    const double deg_initial = 0.0;
    const double tx_initial = 0.0;
    const double ty_initial = 0.0;

    double deg = deg_initial;
    double tx = tx_initial;
    double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
	for(size_t i =0 ; i< points_p.size(); i++){
	    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<RegistrationCostFunction,1,1,1,1>(new RegistrationCostFunction(points_p[i],points_q[i],weights[i]));
	    problem.AddResidualBlock(cost_function, nullptr,&deg,&tx,&ty);
	}


	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// TODO: Output the final values of the translation and rotation (in degree)
    std::cout << "Initial degree: " << deg_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
    std::cout << "Final degree: " << deg << "\ttx: " << tx << "\tty: " << ty << std::endl;

	return 0;
}