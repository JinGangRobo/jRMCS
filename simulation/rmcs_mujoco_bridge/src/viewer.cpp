#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <QApplication>
#include <QCloseEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QOpenGLFunctions>
#include <QOpenGLWindow>
#include <QSignalBlocker>
#include <QSlider>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "mujoco_model.hpp"

namespace rmcs_mujoco_bridge {
namespace {

class ModelWindow final : public QOpenGLWindow {
public:
    ModelWindow() {
        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&options_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&renderer_);
    }

    ~ModelWindow() override { ReleaseRenderer(); }

    void ReleaseRenderer() {
        ready_to_draw_ = false;
        if (!renderer_ready_)
            return;
        if (isExposed()) {
            makeCurrent();
            mjr_freeContext(&renderer_);
            doneCurrent();
        }
        mjv_freeScene(&scene_);
        renderer_ready_ = false;
    }

    void LoadDescription(const std::string& description) {
        model_ = LoadModel(description);
        data_ = MakeData(*model_);
        mjv_defaultFreeCamera(model_.get(), &camera_);
        camera_.azimuth = 135;
        camera_.elevation = -22;
    }

    const mjModel* model() const { return model_.get(); }

    void SetJointPosition(
        const std::array<int, kJointCount>& address,
        const std::array<double, kJointCount>& position) {
        for (std::size_t i = 0; i < kJointCount; ++i)
            data_->qpos[address[i]] = position[i];
        // 只更新渲染模型的正运动学，物理步进由 RMCS 进程负责。
        mj_kinematics(model_.get(), data_.get());
        mj_comPos(model_.get(), data_.get());
        ready_to_draw_ = true;
        update();
    }

protected:
    void paintGL() override {
        if (!ready_to_draw_) {
            auto* gl = context()->functions();
            gl->glClearColor(0.12f, 0.15f, 0.19f, 1.0f);
            gl->glClear(GL_COLOR_BUFFER_BIT);
            return;
        }
        if (!renderer_ready_) {
            mjv_makeScene(model_.get(), &scene_, 1000);
            mjr_makeContext(model_.get(), &renderer_, mjFONTSCALE_100);
            renderer_ready_ = true;
        }
        mjv_updateScene(
            model_.get(), data_.get(), &options_, nullptr, &camera_, mjCAT_ALL, &scene_);
        const mjrRect viewport{
            0, 0, qRound(width() * devicePixelRatio()), qRound(height() * devicePixelRatio())};
        mjr_render(viewport, &scene_, &renderer_);
    }

    void mousePressEvent(QMouseEvent* event) override { last_mouse_ = event->pos(); }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (!renderer_ready_ || height() == 0)
            return;
        const auto delta = event->pos() - last_mouse_;
        last_mouse_ = event->pos();
        const auto action =
            event->buttons().testFlag(Qt::RightButton) ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V;
        mjv_moveCamera(
            model_.get(), action, double(delta.x()) / height(), double(delta.y()) / height(),
            &camera_);
        update();
    }

    void wheelEvent(QWheelEvent* event) override {
        if (!renderer_ready_)
            return;
        mjv_moveCamera(
            model_.get(), mjMOUSE_ZOOM, 0.0, -0.0005 * event->angleDelta().y(), &camera_);
        update();
    }

private:
    ModelPtr model_{nullptr, mj_deleteModel};
    DataPtr data_{nullptr, mj_deleteData};
    mjvCamera camera_{};
    mjvOption options_{};
    mjvScene scene_{};
    mjrContext renderer_{};
    QPoint last_mouse_;
    bool ready_to_draw_ = false;
    bool renderer_ready_ = false;
};

class Viewer final : public QWidget {
public:
    Viewer()
        : node_(std::make_shared<rclcpp::Node>("mujoco_viewer")) {
        setWindowTitle("JGarm — MuJoCo");
        resize(1120, 650);
        auto* layout = new QHBoxLayout(this);
        view_ = new ModelWindow;
        layout->addWidget(QWidget::createWindowContainer(view_, this), 1);

        auto* panel = new QWidget(this);
        panel->setFixedWidth(300);
        auto* controls = new QVBoxLayout(panel);
        status_ = new QLabel("等待仿真反馈", panel);
        status_->setWordWrap(true);
        controls->addWidget(new QLabel("目标角与实际角（rad）", panel));
        controls->addWidget(status_);
        for (std::size_t i = 0; i < kJointCount; ++i) {
            auto& axis = axes_[i];
            axis.name = "joint" + std::to_string(i + 1);
            axis.label = new QLabel(QString("J%1").arg(i + 1), panel);
            axis.slider = new QSlider(Qt::Horizontal, panel);
            axis.slider->setObjectName(QString("joint%1_target").arg(i + 1));
            axis.slider->setEnabled(false);
            axis.slider->setPageStep(100);
            controls->addWidget(axis.label);
            controls->addWidget(axis.slider);
            connect(axis.slider, &QSlider::valueChanged, this, [this, i](int value) {
                command_.data[i] = ToAngle(value);
                command_pub_->publish(command_);
            });
        }
        controls->addStretch();
        controls->addWidget(new QLabel("左键旋转 · 右键平移 · 滚轮缩放", panel));
        layout->addWidget(panel);

        command_.data.resize(kJointCount);
        command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/mujoco/target_position", rclcpp::QoS(1));
        description_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/mujoco/robot_description", rclcpp::QoS(1).transient_local().reliable(),
            [this](const std_msgs::msg::String& message) { OnDescription(message.data); });
        state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/mujoco/joint_states", rclcpp::QoS(1),
            [this](const sensor_msgs::msg::JointState& state) { OnJointState(state); });
        executor_.add_node(node_);

        connect(&timer_, &QTimer::timeout, this, [this] { UpdateUi(); });
        timer_.start(16);
    }

protected:
    void closeEvent(QCloseEvent* event) override {
        timer_.stop();
        // 在父窗口仍可用时释放原生 OpenGL 子窗口。
        delete view_;
        view_ = nullptr;
        QWidget::closeEvent(event);
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr double kSliderScale = 1000.0;

    struct Axis {
        std::string name;
        QLabel* label;
        QSlider* slider;
    };

    static double ToAngle(int value) { return value / kSliderScale; }

    void OnDescription(const std::string& description) {
        if (model_ready_)
            return;
        try {
            view_->LoadDescription(description);
            const auto& model = *view_->model();
            for (std::size_t i = 0; i < kJointCount; ++i) {
                const auto& axis = axes_[i];
                const int joint = mj_name2id(&model, mjOBJ_JOINT, axis.name.c_str());
                if (joint < 0 || model.jnt_type[joint] != mjJNT_HINGE || !model.jnt_limited[joint])
                    throw std::runtime_error("Invalid model joint: " + axis.name);
                address_[i] = model.jnt_qposadr[joint];
                const QSignalBlocker block(axis.slider);
                axis.slider->setRange(
                    std::ceil((model.jnt_range[2 * joint] + kTargetMargin) * kSliderScale),
                    std::floor((model.jnt_range[2 * joint + 1] - kTargetMargin) * kSliderScale));
            }
            model_ready_ = true;
        } catch (const std::exception& e) {
            status_->setText(QString::fromUtf8(e.what()));
            RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
        }
    }

    void OnJointState(const sensor_msgs::msg::JointState& state) {
        if (!model_ready_ || state.name.size() != kJointCount
            || state.position.size() != kJointCount)
            return;
        std::array<double, kJointCount> position;
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const auto joint = std::find(state.name.begin(), state.name.end(), axes_[i].name);
            if (joint == state.name.end())
                return;
            const double angle = state.position[joint - state.name.begin()];
            if (!std::isfinite(angle))
                return;
            position[i] = angle;
        }
        if (!has_feedback_) {
            for (std::size_t i = 0; i < kJointCount; ++i) {
                auto* slider = axes_[i].slider;
                const QSignalBlocker block(slider);
                slider->setValue(qRound(position[i] * kSliderScale));
                command_.data[i] = position[i];
            }
            has_feedback_ = true;
        }
        position_ = position;
        last_feedback_ = Clock::now();
        view_->SetJointPosition(address_, position_);
    }

    void UpdateUi() {
        if (!rclcpp::ok()) {
            close();
            return;
        }
        executor_.spin_some();
        if (!has_feedback_)
            return;
        const bool fresh = Clock::now() - last_feedback_ < std::chrono::milliseconds(500);
        const bool connected = command_pub_->get_subscription_count() != 0;
        status_->setText(fresh && connected ? "仿真运行中" : "等待仿真连接");
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const auto& axis = axes_[i];
            axis.slider->setEnabled(fresh && connected);
            axis.label->setText(QString("J%1    目标 %2    实际 %3")
                                    .arg(i + 1)
                                    .arg(command_.data[i], 0, 'f', 3)
                                    .arg(position_[i], 0, 'f', 3));
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    ModelWindow* view_;
    QLabel* status_;
    std::array<Axis, kJointCount> axes_{};
    std::array<int, kJointCount> address_{};
    std::array<double, kJointCount> position_{};
    std_msgs::msg::Float64MultiArray command_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    QTimer timer_;
    Clock::time_point last_feedback_{};
    bool model_ready_ = false;
    bool has_feedback_ = false;
};

} // namespace
} // namespace rmcs_mujoco_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    QSurfaceFormat format;
    format.setVersion(2, 1);
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);
    QApplication app(argc, argv);
    rmcs_mujoco_bridge::Viewer viewer;
    viewer.show();
    const int result = app.exec();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
