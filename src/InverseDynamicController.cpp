#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

// ============================================================================
// ESTRUCTURAS SIMPLIFICADAS
// ============================================================================
struct Position {
    std::vector<double> X;     // [x, y, z, yaw]
    std::vector<double> dX;    // velocidades [vx, vy, vz, wyaw]
    std::vector<double> ddX;   // aceleraciones [ax, ay, az, ayaw]
    std::vector<double> Xd;    // referencia posición/orientación
    std::vector<double> dXd;   // referencia velocidades
    std::vector<double> ddXd;  // referencia aceleraciones
};

struct Parameters {
    std::vector<double> Model_simp;
    std::vector<double> uSat;
    double g;
    double Altmax;
};

struct SC {
    std::vector<double> Ud = {0, 0, 0, 0, 0, 0};
    std::vector<double> Ur = {0, 0, 0, 0};
    rclcpp::Time tcontrol;
};

// ============================================================================
// CLASE BEBOP
// ============================================================================
class Bebop
{
public:
    explicit Bebop(rclcpp::Node *node)
        : pNode(node), dt(1.0 / 30.0), ref_received(false)
    {
        pPos.X.assign(4, 0.0);
        pPos.dX.assign(4, 0.0);
        pPos.ddX.assign(4, 0.0);
        pPos.Xd.assign(4, 0.0);
        pPos.dXd.assign(4, 0.0);
        pPos.ddXd.assign(4, 0.0);

        last_vel.assign(4, 0.0);
        last_ref.assign(8, 0.0);
        first_read = true;
        first_ref = true;

        pPar.Model_simp = {0.8417, 0.18227, 0.8354, 0.17095,
                           3.966, 4.001, 9.8524, 4.7295};
        pPar.g = 9.8;
        pPar.Altmax = 2000.0;
        pPar.uSat = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

        subOdom = pNode->create_subscription<nav_msgs::msg::Odometry>(
            "/bebop/odom", 10,
            std::bind(&Bebop::odomCallback, this, std::placeholders::_1));

        subRef = pNode->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/bebop/ref_vec", 10,
            std::bind(&Bebop::refCallback, this, std::placeholders::_1));

        pubCmd = pNode->create_publisher<geometry_msgs::msg::Twist>(
            "/safe_bebop/cmd_vel", 10);
    }

    // ------------------------------------------------------------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pOdom = msg;
    }

    // ------------------------------------------------------------------------
    void refCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 8)
        {
            RCLCPP_WARN(pNode->get_logger(), "Referencia incorrecta: se esperaban 8 elementos");
            return;
        }

        ref_received = true;

        for (int i = 0; i < 4; ++i)
        {
            pPos.Xd[i] = msg->data[i];
            pPos.dXd[i] = msg->data[i + 4];
        }

        // Calcular aceleración deseada (derivada discreta)
        if (!first_ref)
        {
            for (int i = 0; i < 4; ++i)
                pPos.ddXd[i] = (msg->data[i + 4] - last_ref[i + 4]) / dt;
        }
        else
        {
            std::fill(pPos.ddXd.begin(), pPos.ddXd.end(), 0.0);
            first_ref = false;
        }

        last_ref = msg->data;
    }

    // ------------------------------------------------------------------------
    void rGetSensorData()
    {
        if (!pOdom) return;

        const auto &q = pOdom->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

        pPos.X = {pOdom->pose.pose.position.x,
                  pOdom->pose.pose.position.y,
                  pOdom->pose.pose.position.z,
                  yaw};

        // --- Calcular velocidades ---
        std::vector<double> new_dX(4);
        new_dX[0] = pOdom->twist.twist.linear.x;
        new_dX[1] = pOdom->twist.twist.linear.y;
        new_dX[2] = pOdom->twist.twist.linear.z;

        static double last_yaw = yaw;
        static bool first = true;
        double yaw_rate = 0.0;
        if (!first)
        {
            double dyaw = yaw - last_yaw;
            if (dyaw > M_PI) dyaw -= 2 * M_PI;
            else if (dyaw < -M_PI) dyaw += 2 * M_PI;
            yaw_rate = dyaw / dt;
        }
        else first = false;

        last_yaw = yaw;
        new_dX[3] = yaw_rate;
        pPos.dX = new_dX;

        // --- Calcular aceleraciones ---
        if (first_read)
        {
            std::fill(pPos.ddX.begin(), pPos.ddX.end(), 0.0);
            first_read = false;
        }
        else
        {
            for (int i = 0; i < 4; ++i)
                pPos.ddX[i] = (pPos.dX[i] - last_vel[i]) / dt;
        }
        last_vel = pPos.dX;
    }

    // ------------------------------------------------------------------------
    void cInverseDynamicController_Compensador(const std::vector<double> &gains)
    {
        if (!ref_received)
            return; // No hace nada si no hay referencia

        std::vector<double> g = gains;
        if (g.empty())
            g = {2, 2, 3, 1.5,
                 2, 2, 1.8, 5,
                 1, 1, 1, 1.5,
                 1, 1, 1, 1};

        Eigen::Matrix4d Ku = Eigen::Matrix4d::Zero();
        Eigen::Matrix4d Kv = Eigen::Matrix4d::Zero();

        Ku << pPar.Model_simp[0], 0, 0, 0,
              0, pPar.Model_simp[2], 0, 0,
              0, 0, pPar.Model_simp[4], 0,
              0, 0, 0, pPar.Model_simp[6];

        Kv << pPar.Model_simp[1], 0, 0, 0,
              0, pPar.Model_simp[3], 0, 0,
              0, 0, pPar.Model_simp[5], 0,
              0, 0, 0, pPar.Model_simp[7];

        Eigen::Matrix4d Ksp, Ksd, Kp, Kd;
        Ksp << g[0], 0, 0, 0,
               0, g[1], 0, 0,
               0, 0, g[2], 0,
               0, 0, 0, g[3];

        Ksd << g[4], 0, 0, 0,
               0, g[5], 0, 0,
               0, 0, g[6], 0,
               0, 0, 0, g[7];

        Kp << g[8], 0, 0, 0,
              0, g[9], 0, 0,
              0, 0, g[10], 0,
              0, 0, 0, g[11];

        Kd << g[12], 0, 0, 0,
              0, g[13], 0, 0,
              0, 0, g[14], 0,
              0, 0, 0, g[15];

        Eigen::Vector4d X, dX, ddX, Xd, dXd, ddXd;
        X    << pPos.X[0],  pPos.X[1],  pPos.X[2],  pPos.X[3];
        dX   << pPos.dX[0], pPos.dX[1], pPos.dX[2], pPos.dX[3];
        ddX  << pPos.ddX[0], pPos.ddX[1], pPos.ddX[2], pPos.ddX[3];
        Xd   << pPos.Xd[0],  pPos.Xd[1],  pPos.Xd[2],  pPos.Xd[3];
        dXd  << pPos.dXd[0], pPos.dXd[1], pPos.dXd[2], pPos.dXd[3];
        ddXd << pPos.ddXd[0], pPos.ddXd[1], pPos.ddXd[2], pPos.ddXd[3];

        Eigen::Vector4d Xtil = Xd - X;
        if (std::abs(Xtil[3]) > M_PI)
            Xtil[3] += (Xtil[3] > 0 ? -2 * M_PI : 2 * M_PI);

        Eigen::Vector4d Ucw_ant(pSC.Ur[0], pSC.Ur[1], pSC.Ur[2], pSC.Ur[3]);
        Eigen::Vector4d Ucw = dXd + Ksp * ((Kp * Xtil).array().tanh()).matrix();
        const double dt_control = 1.0 / 30.0;
        Eigen::Vector4d dUcw = (Ucw - Ucw_ant) / std::max(dt_control, 1e-3);
        pSC.Ur = {Ucw[0], Ucw[1], Ucw[2], Ucw[3]};

        double psi = X[3];
        Eigen::Matrix4d F;
        F << std::cos(psi), -std::sin(psi), 0, 0,
             std::sin(psi),  std::cos(psi), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

        Eigen::Vector4d Udw = (F * Ku).inverse() * (dUcw + Ksd * (Ucw - dX) + Kv * dX);

        std::vector<double> k_max = {1.1, 1.1, 1.0, 1};
        std::vector<double> k_min = {0.2, 0.2, 0.8, 1};
        std::vector<double> n_exp = {2.0, 2.0, 2.0, 2.0};
        std::vector<double> e0    = {0.02, 0.02, 0.02, 0.02};

        Eigen::Vector4d Kdyn;
        for (int i = 0; i < 4; ++i)
        {
            double e_abs = std::abs(Xtil[i]);
            Kdyn[i] = k_min[i] + (k_max[i] - k_min[i]) / (1.0 + std::pow(e_abs / e0[i], n_exp[i]));
        }

        Udw = Kdyn.asDiagonal() * Udw;
        pSC.Ud[0] = Udw[0];
        pSC.Ud[1] = Udw[1];
        pSC.Ud[2] = Udw[2];
        pSC.Ud[5] = Udw[3];
    }

    // ------------------------------------------------------------------------
    void rSendControlSignals()
    {
        if (!ref_received)
            return;  // No publicar si no hay referencia

        geometry_msgs::msg::Twist cmd_sat;
        cmd_sat.linear.x  = std::clamp(pSC.Ud[0], -pPar.uSat[0], pPar.uSat[0]);
        cmd_sat.linear.y  = std::clamp(pSC.Ud[1], -pPar.uSat[1], pPar.uSat[1]);
        cmd_sat.linear.z  = std::clamp(pSC.Ud[2], -pPar.uSat[2], pPar.uSat[2]);
        cmd_sat.angular.z = std::clamp(pSC.Ud[5], -pPar.uSat[5], pPar.uSat[5]);
        pubCmd->publish(cmd_sat);
    }

    Position pPos;
    Parameters pPar;
    SC pSC;
    bool ref_received;

private:
    rclcpp::Node *pNode;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subRef;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmd;
    nav_msgs::msg::Odometry::SharedPtr pOdom;
    std::vector<double> last_vel, last_ref;
    bool first_read, first_ref;
    const double dt;
};

// ============================================================================
// NODO PRINCIPAL
// ============================================================================
class NeroDroneNode : public rclcpp::Node
{
public:
    NeroDroneNode() : Node("nero_drone_node")
    {
        RCLCPP_INFO(this->get_logger(), "Nero Drone Node iniciado (esperando referencia externa)");
        drone = std::make_shared<Bebop>(this);
        timer = this->create_wall_timer(33ms, std::bind(&NeroDroneNode::controlLoop, this));
    }

private:
    std::shared_ptr<Bebop> drone;
    rclcpp::TimerBase::SharedPtr timer;

    void controlLoop()
    {
        drone->rGetSensorData();
        drone->cInverseDynamicController_Compensador({});
        drone->rSendControlSignals();
    }
};

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NeroDroneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
