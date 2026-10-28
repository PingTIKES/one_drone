#include "tf_shift_panel.h"
#include <pluginlib/class_list_macros.hpp>

#include <QFormLayout>

#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace tf_shift_panel
{

    DirectionalControlWidget::DirectionalControlWidget(QWidget *parent)
        : QWidget(parent)
    {

        setFixedSize(200, 200);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);
    }

    void DirectionalControlWidget::forcePublishValues(double x, double y, double yaw)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return;
        setDirection(x, y, yaw);
        emit directionChanged();
        setFocus();
    }

    void DirectionalControlWidget::paintEvent(QPaintEvent *event)
    {
        Q_UNUSED(event);
        QPainter painter(this);
        painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

        QPoint centerPoint = rect().center();
        double radius = (std::min(width(), height()) - 20) / 2.0;
        drawRect_ = QRectF(centerPoint.x() - radius, centerPoint.y() - radius,
                           radius * 2, radius * 2);
        double arcHeight = radius / 2.0;

        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor("#EAEAEA"));
        painter.drawEllipse(drawRect_);

        transShaped[0] = gradientArc(45.0, 90.0, arcHeight);
        transShaped[1] = gradientArc(225.0, 90.0, arcHeight);
        transShaped[2] = gradientArc(135.0, 90.0, arcHeight);
        transShaped[3] = gradientArc(315.0, 90.0, arcHeight);

        painter.setBrush(QColor("#D0D0D0"));
        for (auto &path : transShaped)
        {
            painter.drawPath(path);
        }

        double centerRadius = radius / 2.0;
        centerRect_ = QRectF(centerPoint.x() - centerRadius,
                             centerPoint.y() - centerRadius,
                             centerRadius * 2, centerRadius * 2);

        ccwPath_ = createSemiCircle(false);
        cwPath_ = createSemiCircle(true);

        painter.setBrush((pressedBtn_ == PressBtnType::CCW) ? QColor(0, 0, 0, 63) : QColor("#D0D0D0"));
        painter.drawPath(ccwPath_);

        painter.setBrush((pressedBtn_ == PressBtnType::CW) ? QColor(0, 0, 0, 63) : QColor("#D0D0D0"));
        painter.drawPath(cwPath_);

        painter.setPen(QPen(Qt::black, 2));
        QFont font = painter.font();
        font.setPixelSize(16);
        painter.setFont(font);

        painter.drawText(QRectF(drawRect_.x(), centerRect_.y(), radius / 2, radius),
                         Qt::AlignCenter, "<");
        painter.drawText(QRectF(centerRect_.x(), drawRect_.y(), radius, radius / 2),
                         Qt::AlignCenter, "^");
        painter.drawText(QRectF(centerRect_.topRight().x(), centerRect_.topRight().y(),
                                radius / 2, radius),
                         Qt::AlignCenter, ">");
        painter.drawText(QRectF(centerRect_.bottomLeft().x(),
                                centerRect_.bottomLeft().y(), radius, radius / 2),
                         Qt::AlignCenter, "v");

        font.setPixelSize(24);
        painter.setFont(font);
        painter.drawText(cwPath_.boundingRect(), Qt::AlignCenter, "↻");
        painter.drawText(ccwPath_.boundingRect(), Qt::AlignCenter, "↺");

        if (pressedBtn_ != PressBtnType::None &&
            (pressedBtn_ != PressBtnType::CCW && pressedBtn_ != PressBtnType::CW))
        {
            painter.setBrush(QColor(0, 0, 0, 63));
            int index = static_cast<int>(pressedBtn_) - 3;
            if (index >= 0 && index < 4)
            {
                painter.drawPath(transShaped[index]);
            }
        }
    }

    void DirectionalControlWidget::mousePressEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            QPoint point = event->pos();

            if (isPointInPath(point, ccwPath_))
            {
                pressedBtn_ = PressBtnType::CCW;
                createTwist(2, 0.017);
                emit directionChanged();
            }
            else if (isPointInPath(point, cwPath_))
            {
                pressedBtn_ = PressBtnType::CW;
                createTwist(2, -0.017);
                emit directionChanged();
            }
            else if (isPointInCircle(point, drawRect_.toRect()))
            {
                QPoint center = drawRect_.center().toPoint();
                double angle = atan2(point.y() - center.y(), point.x() - center.x());
                angle = -angle * (180.0 / M_PI);
                if (angle < 0)
                    angle += 360.0;

                if (angle < 45 || angle >= 315)
                {
                    pressedBtn_ = PressBtnType::Right;
                    createTwist(1, -0.05);
                    emit directionChanged();
                }
                else if (angle >= 45 && angle < 135)
                {
                    pressedBtn_ = PressBtnType::Up;
                    createTwist(0, 0.05);
                    emit directionChanged();
                }
                else if (angle >= 135 && angle < 225)
                {
                    pressedBtn_ = PressBtnType::Left;
                    createTwist(1, 0.05);
                    emit directionChanged();
                }
                else if (angle >= 225 && angle < 315)
                {
                    pressedBtn_ = PressBtnType::Down;
                    createTwist(0, -0.05);
                    emit directionChanged();
                }
            }
            update();
        }
        QWidget::mousePressEvent(event);
    }

    void DirectionalControlWidget::keyPressEvent(QKeyEvent *event)
    {
        switch (event->key())
        {
        case Qt::Key_W:
            pressedBtn_ = PressBtnType::Up;
            createTwist(0, 0.05);
            emit directionChanged();
            break;
        case Qt::Key_S:
            pressedBtn_ = PressBtnType::Down;
            createTwist(0, -0.05);
            emit directionChanged();
            break;
        case Qt::Key_A:
            pressedBtn_ = PressBtnType::Left;
            createTwist(1, 0.05);
            emit directionChanged();
            break;
        case Qt::Key_D:
            pressedBtn_ = PressBtnType::Right;
            createTwist(1, -0.05);
            emit directionChanged();
            break;
        case Qt::Key_Q:
            pressedBtn_ = PressBtnType::CCW;
            createTwist(2, 0.017);
            emit directionChanged();
            break;
        case Qt::Key_E:
            pressedBtn_ = PressBtnType::CW;
            createTwist(2, -0.017);
            emit directionChanged();
            break;
        default:
            QWidget::keyPressEvent(event);
            return;
        }

        update();
    }

    void DirectionalControlWidget::mouseReleaseEvent(QMouseEvent *event)
    {
        if (pressedBtn_ != PressBtnType::None)
        {
            pressedBtn_ = PressBtnType::None;
            update();
        }
        QWidget::mouseReleaseEvent(event);
    }

    void DirectionalControlWidget::keyReleaseEvent(QKeyEvent *event)
    {
        Q_UNUSED(event);
        pressedBtn_ = PressBtnType::None;
        update();
    }

    bool DirectionalControlWidget::isPointInCircle(const QPoint &point, const QRect &rect)
    {
        QPoint center = rect.center();
        int dx = point.x() - center.x();
        int dy = point.y() - center.y();
        return (dx * dx + dy * dy) <= (rect.width() / 2 * rect.width() / 2);
    }

    bool DirectionalControlWidget::isPointInPath(const QPoint &point, const QPainterPath &path)
    {
        return path.contains(point);
    }

    QPainterPath DirectionalControlWidget::gradientArc(double startAngle, double angleLength, double arcHeight)
    {
        QPainterPath path;
        path.arcMoveTo(drawRect_, startAngle);
        path.arcTo(drawRect_, startAngle, angleLength);

        QPainterPath innerPath;
        innerPath.addEllipse(drawRect_.adjusted(arcHeight, arcHeight, -arcHeight, -arcHeight));

        return path - innerPath;
    }

    QPainterPath DirectionalControlWidget::createSemiCircle(bool left)
    {
        QPainterPath path;
        QRectF baseRect = centerRect_.adjusted(2, 2, -2, -2);

        if (left)
        {
            path.arcMoveTo(baseRect, 90);
            path.arcTo(baseRect, 90, 180);
        }
        else
        {
            path.arcMoveTo(baseRect, 270);
            path.arcTo(baseRect, 270, 180);
        }
        path.closeSubpath();
        return path;
    }

    void DirectionalControlWidget::createTwist(int axis, double delta)
    {
        directionArray[axis] += delta;
    }

    void DirectionalControlWidget::setDirection(double x, double y, double yaw)
    {
        directionArray[0] = x;
        directionArray[1] = y;
        directionArray[2] = yaw;
    }

    void DirectionalControlWidget::readDirection(double (&values)[3]) const
    {
        std::copy(directionArray, directionArray + 3, values);
    }

    Rviz2Panel::Rviz2Panel(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        auto *layout = new QVBoxLayout(this);
        controlWidget_ = new DirectionalControlWidget(this);
        xSpinBox_ = new QDoubleSpinBox(this);
        ySpinBox_ = new QDoubleSpinBox(this);
        rotationSpinBox_ = new QDoubleSpinBox(this);
        forceYamlButton_ = new QPushButton("强制发布", this);
        forceYamlButton_->setToolTip("将 X/Y/Rotation 写入 map→odom；Rotation 单位为弧度");
        for (QDoubleSpinBox *box : {xSpinBox_, ySpinBox_, rotationSpinBox_})
        {
            box->setRange(-10000.0, 10000.0);
            box->setDecimals(6);
            box->setSingleStep(0.05);
        }
        rotationSpinBox_->setSingleStep(0.017);
        auto *form = new QFormLayout();
        form->addRow("X (m)", xSpinBox_);
        form->addRow("Y (m)", ySpinBox_);
        form->addRow("Rotation (rad)", rotationSpinBox_);
        layout->addWidget(controlWidget_);
        layout->addLayout(form);
        layout->addWidget(forceYamlButton_);
        layout->addStretch();
        controlWidget_->setFocus();
        connect(forceYamlButton_, &QPushButton::clicked, this, [this]() {
            controlWidget_->forcePublishValues(
                xSpinBox_->value(), ySpinBox_->value(), rotationSpinBox_->value());
        });
        connect(controlWidget_, &DirectionalControlWidget::directionChanged,
                this, &Rviz2Panel::publishAlignment);
        // ROS callbacks can run outside the GUI thread.
        connect(this, &Rviz2Panel::alignmentReceived, this,
                [this](double x, double y, double yaw) {
                    controlWidget_->setDirection(x, y, yaw);
                    xSpinBox_->setValue(x);
                    ySpinBox_->setValue(y);
                    rotationSpinBox_->setValue(yaw);
                }, Qt::QueuedConnection);
    }

    void Rviz2Panel::onInitialize()
    {
        auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
        if (!abstraction)
        {
            return;
        }
        node_ = abstraction->get_raw_node();
        alignmentPub_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(
            "/map_odom/set", rclcpp::QoS(10));
        alignmentSub_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
            "/map_odom/current", rclcpp::QoS(1).transient_local(),
            [this](const geometry_msgs::msg::Pose2D::SharedPtr value) {
                if (std::isfinite(value->x) && std::isfinite(value->y) && std::isfinite(value->theta))
                    emit alignmentReceived(value->x, value->y, value->theta);
            });
    }

    void Rviz2Panel::publishAlignment()
    {
        if (!alignmentPub_)
        {
            return;
        }
        double alignment_[3];
        controlWidget_->readDirection(alignment_);
        geometry_msgs::msg::Pose2D msg;
        msg.x = alignment_[0];
        msg.y = alignment_[1];
        msg.theta = alignment_[2];
        alignmentPub_->publish(msg);
    }

}

PLUGINLIB_EXPORT_CLASS(tf_shift_panel::Rviz2Panel, rviz_common::Panel)
