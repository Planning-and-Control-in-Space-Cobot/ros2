#include "attitude_controller_cpp/controller.hpp"
#include "attitude_controller_cpp/libbnpy/include/npy.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

                                         Controller::Controller(int N, double dt, int frequency)
    : Node("AttitudeController"),
N(N), dt(dt)
{
  // Declare the parameters needed for the optimization problem
  this->declare_parameter("N", N);
  this->declare_parameter("dt", dt);
  this->declare_parameter("frequency", frequency);
  this->declare_parameter("imu_topic", "/mavros/imu/data");
  this->declare_parameter("desired_orientation_topic", "/Control/PredictiveController/Attitude/desired_orientation");

  // Get the parameters to see if they have been set in a launch file or not
  N = this->get_parameter("N").as_int();
  dt = this->get_parameter("dt").as_double();
  frequency = this->get_parameter("frequency").as_int();
  std::string imu_topic = this->get_parameter("imu_topic").as_string();
  std::string desired_orientation_topic = this->get_parameter("desired_orientation_topic").as_string();

  pipeline = std::make_shared<ControllerPipeline>();

  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), std::bind(&ControllerPipeline::updateImuMsg, pipeline, std::placeholders::_1));

  this->desired_orientation_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      desired_orientation_topic, 10, std::bind(&ControllerPipeline::updateDesiredOrientation, pipeline, std::placeholders::_1));

  // TODO:  Should be changed to parameters in order to be able to change the system parameters as easily as needed
  // Get all the system paramters and convert them to C++ types, in this case from numpy to eigen
  std::string package_name = "attitude_controller_cpp";
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);

  // Create the variables for the optimization problem
  std::string J_file_path = {"system_parameters/J.npy"};
  std::string c_file_path = {"system_parameters/c.npy"};
  std::string g_file_path = {"system_parameters/g.npy"};
  std::string A_file_path = {"system_parameters/A.npy"};

  J_file_path = package_share_directory + "/" + J_file_path;
  c_file_path = package_share_directory + "/" + c_file_path;
  g_file_path = package_share_directory + "/" + g_file_path;
  A_file_path = package_share_directory + "/" + A_file_path;

  // Load the system parameters
  npy::npy_data J = npy::read_npy<double>(J_file_path);
  npy::npy_data c = npy::read_npy<double>(c_file_path);
  npy::npy_data g = npy::read_npy<double>(g_file_path);
  npy::npy_data A = npy::read_npy<double>(A_file_path);

  // Extract the lower part of the actuation matrix, the part that is related to the momentum of the robot, and not the position
  std::vector<double> A_momentum(A.data.end() - 18, A.data.end());

  this->J_ = vecToMatrix3d(J.data);
  this->c_ = vecToeigVector(c.data);
  this->g_ = vecToeigVector(g.data);
  this->A_ = vecToMatrix3_6(A_momentum);

  this->setup();

  // Create timer to run the optimization problem
  this->step_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / frequency), std::bind(&Controller::step, this));
  std::cout << 1000 / frequency << std::endl;
}

Controller::~Controller()
{
}

void Controller::setup()
{
  if (this->setup_done)
  {
    std::cout << "leaving setup stage " << std::endl;
    return;
  }

  // Declare all the
  this->opti = new casadi::Opti();

  // parameter required for the optimization problem
  this->w0 = this->opti->parameter(3, 1);
  this->w_dot0 = this->opti->parameter(3, 1);
  this->q0 = this->opti->parameter(4, 1);
  this->desired_rotation = this->opti->parameter(4, 1);

  // Systems parameter
  this->J = this->opti->parameter(3, 3);
  this->g = this->opti->parameter(3, 1);
  this->A = this->opti->parameter(3, 6);
  this->c = this->opti->parameter(3, 1);

  // Variables
  this->w = this->opti->variable(3, N);
  this->w_dot = this->opti->variable(3, N);
  this->q = this->opti->variable(4, N);
  this->u = this->opti->variable(6, N);

  // Setup system parameter
  this->opti->set_value(this->J, this->eigenToCasadi(this->J_));
  this->opti->set_value(this->g, this->eigenToCasadi(this->g_));
  this->opti->set_value(this->A, this->eigenToCasadi(this->A_));
  this->opti->set_value(this->c, this->eigenToCasadi(this->c_));

  // Setup initial conditions
  this->opti->set_value(this->w0, {0, 0, 0});
  this->opti->set_value(this->w_dot0, {0, 0, 0});
  this->opti->set_value(this->q0, {0, 0, 0, 1});
  this->opti->set_value(this->desired_rotation, {0, 0, 0, 1});

  this->opti->subject_to(this->q(casadi::Slice(), 0) == this->q0);
  this->opti->subject_to(this->w(casadi::Slice(), 0) == this->w0);
  this->opti->subject_to(this->w_dot(casadi::Slice(), 0) == this->w_dot0);

  for (int i = 0; i < N; i++)
  {
    // Make sure the norm of the quaternion is 1 for all the horizon, this will help to make sure we dont go to bad rotations
    this->opti->set_initial(this->q(casadi::Slice(), i), {0, 0, 0, 1}); // initial guess for the quaternion must have norm 1
    // this->opti->subject_to(norm_2(this->q(casadi::Slice(), i)) == 1);

    // Add boundaries for value of the actuation
    for (int j = 0; j < 6; j++)
    {
      this->opti->subject_to(this->opti->bounded(-2, this->u(j, i), 2));
    }
  }

  for (int i = 0; i < this->N; i++)
  {
    this->rotation.fromQuat(this->q(casadi::Slice(), i));
    auto c_r = this->rotation.toRotationMatrix();

    // System dynamics
    this->opti->subject_to(
        (casadi::MX::mtimes(J, this->w_dot(casadi::Slice(), i)) +
         casadi::MX::mtimes(casadi::MX::skew(this->w(casadi::Slice(), i)), casadi::MX::mtimes(this->J, this->w(casadi::Slice(), i))) -
         casadi::MX::mtimes(casadi::MX::skew(this->c), casadi::MX::mtimes(c_r.T(), this->g))) ==
        casadi::MX::mtimes(this->A, this->u(casadi::Slice(), i)));

    if (i < this->N - 1)
    {
      // Integrate quaternion
      this->opti->subject_to(
          this->q(casadi::Slice(), i + 1) == this->quaternion_integration(this->q(casadi::Slice(), i), this->w(casadi::Slice(), i), this->dt));

      // Integrate angular velocity
      this->opti->subject_to(
          this->w(casadi::Slice(), i + 1) == this->w(casadi::Slice(), i) + this->dt * this->w_dot(casadi::Slice(), i));
    }
  }

  this->rotation.fromQuat(this->desired_rotation);
  auto desired_rot = this->rotation.toRotationMatrix();
  desired_rot = desired_rot.T();

  casadi::MX cost = 0;
  for (int i = 0; i < this->N; i++)
  {
    this->rotation.fromQuat(this->q(casadi::Slice(), i));
    auto c_r = this->rotation.toRotationMatrix();
    // desired rotation is a quat here, need to convert to rotation matrix
    cost += casadi::MX::trace(casadi::MX::eye(3) - casadi::MX::mtimes(c_r, desired_rot));
  }

  casadi::Dict opts;

  // Set IPOPT options
  casadi::Dict ipopt_opts;
  ipopt_opts["print_level"] = 0;          // IPOPT verbosity (0 = silent)
  ipopt_opts["tol"] = 1e-4;               // Tolerance for convergence
  ipopt_opts["mu_strategy"] = "adaptive"; // Adaptive barrier parameter strategy
  ipopt_opts["linear_solver"] = "mumps";  // Linear solver choice (e.g., MUMPS)
  ipopt_opts["acceptable_tol"] = 1e-4;    // Tolerance for acceptable solution
  ipopt_opts["acceptable_iter"] = 10;     // Number of iterations for acceptable solution

  opts["ipopt"] = ipopt_opts;
  // Set other options
  opts["print_time"] = false; // Disable printing time

  this->opti->minimize(cost);

  this->opti->solver("ipopt", opts);

  this->setup_done = true;
}

// TODO change the function to run autonomously and not when called, so it need to get the input arguments from the class variables
void Controller::step()
{
  if (!this->setup_done)
  {
    std::cout << "Setup Not done" << std::endl;
    return;
  }

  if (!this->pipeline->hasImuMsg())
  {
    std::cout << "No IMU msg" << std::endl;
    return;
  }

  // get angular velocity
  auto w0 = this->pipeline->getImuAngularVelocity();
  auto desired_orientation = this->pipeline->getDesiredOrientation();
  auto q0 = this->pipeline->getImuOrientation();

  // Get timestamp_ms
  auto timestamp_ms = this->pipeline->getImuHeader().stamp.sec * 1e9 + this->pipeline->getImuHeader().stamp.nanosec;

  std::vector<double> w_dot0_;

  if (this->prevImuMsgTime_ms == 0.0)
  {
    this->prevImuMsgTime_ms = timestamp_ms;
    this->prevImuAngularVelocity = w0;
    return;
  }
  else
  {
    double diffTime_ms = timestamp_ms - this->prevImuMsgTime_ms;
    for (unsigned int i = 0; i < 3; i++)
    {
      w_dot0_.push_back(w0[i] - this->prevImuAngularVelocity[i] / diffTime_ms);
    }
  }

  if (this->prev_sol.has_value())
  {
    if (this->compareQuaternions(desired_orientation, this->prevDesiredOrientation))
    {
      this->opti->set_initial(this->q, this->prev_sol->value(this->q));
      this->opti->set_initial(this->w, this->prev_sol->value(this->w));
      this->opti->set_initial(this->w_dot, this->prev_sol->value(this->w_dot));
      this->opti->set_initial(this->u, this->prev_sol->value(this->u));
    }
  }

  this->opti->set_value(this->w0, w0);
  this->opti->set_value(this->w_dot0, w_dot0_);
  this->opti->set_value(this->q0, q0);
  this->opti->set_value(this->desired_rotation, desired_orientation);

  // auto time_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  prev_sol = this->opti->solve();

  // auto time_end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  this->prevImuMsgTime_ms = timestamp_ms;
  this->prevImuAngularVelocity = w0;
  this->prevDesiredOrientation = desired_orientation;
  this->prevDesiredOrientation = desired_orientation;
}