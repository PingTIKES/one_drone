#ifndef TF_SHIFT_PANEL_HPP
#define TF_SHIFT_PANEL_HPP

#include <QApplication>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QSharedMemory>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace tf_shift_panel
{

class DirectionalControlWidget : public QWidget
{
    Q_OBJECT
public:
    explicit DirectionalControlWidget(QWidget *parent = nullptr);
    ~DirectionalControlWidget() override;

public slots:
    void forcePublishValues(double x, double y, double yaw);

signals:
    void directionChanged();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    enum class PressBtnType
    {
        None,
        CCW,
        CW,
        Up,
        Down,
        Left,
        Right
    };

    PressBtnType pressedBtn_ = PressBtnType::None;

    QRectF drawRect_;
    QRectF centerRect_;
    QPainterPath transShaped[4];
    QPainterPath ccwPath_;
    QPainterPath cwPath_;

    double directionArray[3] = {0.0, 0.0, 0.0};
    int QSharedMemoryLen = 3;
    QSharedMemory XYR_memory_list;
    bool ensureSharedMemoryAttached();
    bool writeDirection(double x, double y, double yaw);
public:
    bool readDirection(double (&values)[3]);
private:
    void createTwist(int switchXYR, double delta);

    bool isPointInCircle(const QPoint &point, const QRect &rect);
    bool isPointInPath(const QPoint &point, const QPainterPath &path);
    QPainterPath gradientArc(double startAngle, double angleLength, double arcHeight);
    QPainterPath createSemiCircle(bool left);
};

class Rviz2Panel : public rviz_common::Panel
{
    Q_OBJECT
public:
    explicit Rviz2Panel(QWidget *parent = nullptr);
    void onInitialize() override;

private:
    DirectionalControlWidget *controlWidget_{nullptr};
    QDoubleSpinBox *xSpinBox_{nullptr};
    QDoubleSpinBox *ySpinBox_{nullptr};
    QDoubleSpinBox *rotationSpinBox_{nullptr};
    QPushButton *forceYamlButton_{nullptr};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr alignmentPub_;
    double alignment_[3] = {0.0, 0.0, 0.0};
    void publishAlignment();
    bool loadYamlConfig(double &x, double &y, double &yaw) const;
    void loadYamlConfigToInputs();
};

}  // namespace tf_shift_panel

#endif  // TF_SHIFT_PANEL_HPP
