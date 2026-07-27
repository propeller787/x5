#include "operation_control.h"
#include "follow_control.h"
using namespace service::control;

OperationControl::OperationControl()
    : Control("operation")
{
    using namespace std::placeholders;
    // auto global_ = service::Global::getInstance();
    ope_Sce_ = global_->node_->create_service<interface::srv::OperationScenario>(operationSceService, std::bind(&OperationControl::handleOperationScenario, this, _1, _2));
    ope_Sta_ = global_->node_->create_service<interface::srv::OperationStatus>(operationStatusService, std::bind(&OperationControl::handleOperationStatus, this, _1, _2));
    sence_pub_ = global_->node_->create_publisher<interface::msg::VlagoScene>(operationSceTopic, 10);
    object_sub_ = global_->node_->create_subscription<interface::msg::Objects>(perFeatuerTopic, 10, std::bind(&OperationControl::onFeatureResult, this, _1));
    object_elevator_sub_ = global_->node_->create_subscription<interface::msg::Objects>(perDetctorTopic, 10, std::bind(&OperationControl::onDetectorResult, this, _1));
    object_monnai_sub_ = global_->node_->create_subscription<interface::msg::Objects>(perMonnaiTopic, 10, std::bind(&OperationControl::onMonnaiResult, this, _1));
}

// 计算两点间距离的函数
double OperationControl::distance(const Point& p1, const Point& p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// 计算距离point1 0.45米处的点
Point OperationControl::calculatePointAtDistance(const Point& p1, const Point& p2, double distanceFromP1)
{
    double totalDistance = distance(p1, p2);
    double ratio = distanceFromP1 / totalDistance;

    Point result;
    result.x = p1.x + ratio * (p2.x - p1.x);
    result.y = p1.y + ratio * (p2.y - p1.y);

    return result;
}

// 计算距离两点中点2.4米并且垂直于两点线段的点
Point OperationControl::calculatePerpendicularPoint(const Point& p1, const Point& p2, double distanceToCenter, bool symbol)
{
    // 1. 计算中点
    Point midPoint;
    midPoint.x = (p1.x + p2.x) / 2;
    midPoint.y = (p1.y + p2.y) / 2;

    // 2. 计算线段斜率
    double deltaX = p2.x - p1.x;
    double deltaY = p2.y - p1.y;

    // 3. 计算垂直线的斜率
    double perpendicularSlope;
    if (deltaX == 0) {
        // 线段垂直时，垂直线为水平，斜率为0
        perpendicularSlope = 0;
    } else if (deltaY == 0) {
        // 线段水平时，垂直线为竖直，斜率为无穷大，意味着 x 不变
        Point perpendicularPoint = { midPoint.x, midPoint.y + distanceToCenter };
        return perpendicularPoint;
    } else {
        perpendicularSlope = -deltaX / deltaY;
    }

    // 4. 根据垂直线的斜率计算目标点的坐标
    double dx = distanceToCenter / sqrt(1 + perpendicularSlope * perpendicularSlope);
    double dy = perpendicularSlope * dx;

    // 5. 找到距离中点2.4米的两个点之一，取靠近原点的点
    Point point1 = { midPoint.x + dx, midPoint.y + dy };
    Point point2 = { midPoint.x - dx, midPoint.y - dy };

    // 6. 返回离原点近的点
    if (symbol) {
        if (distance({ 0, 0 }, point1) < distance({ 0, 0 }, point2)) {
            return point1;
        } else {
            return point2;
        }
    } else {
        std::cout << " calculatePerpendicularPoint point2 " << point2.x << "," << point2.y << " point1 " << point1.x << "," << point1.y << std::endl;

        if (distance({ 0, 0 }, point1) > distance({ 0, 0 }, point2)) {
            return point1;
        } else {
            return point2;
        }
    }
}

Point OperationControl::calculateCPoint(const Point& A, const Point& B, double distanceBD, double distanceCD)
{
    // 计算 AB 向量
    double dx = B.x - A.x;
    double dy = B.y - A.y;

    // 计算 AB 的长度
    double length = std::sqrt(dx * dx + dy * dy);

    // 单位向量
    double unit_dx = dx / length;
    double unit_dy = dy / length;

    // 计算交点 D（在 B 的延长线上）
    Point D = { B.x + unit_dx * distanceBD, B.y + unit_dy * distanceBD };

    // 垂直于 AB 的单位向量
    double perp_dx = -unit_dy; // 垂直向量
    double perp_dy = unit_dx;

    // 计算 C 点的位置
    Point C1 = { D.x + perp_dx * distanceCD, D.y + perp_dy * distanceCD }; // 在一个方向
    Point C2 = { D.x - perp_dx * distanceCD, D.y - perp_dy * distanceCD }; // 在相反方向
    std::cout << "C2: (" << C2.x << ", " << C2.y << ")\n";
    if (B.y < 0) {
        return C2; // 或返回 C2，根据需要选择
    } else {
        return C1;
    }
}

void OperationControl::onMonnaiResult(const interface::msg::Objects msg)
{
    if (global_->elevator_first_step_ == 3) {
        std::cout << " ============== x " << msg.objects.begin()->x << " y " << msg.objects.begin()->y << "=====================" << std::endl;
    }
}

void OperationControl::onFeatureResult(const interface::msg::Objects msg)
{
    // auto global_ = service::Global::getInstance();
    //  判断是否为两个灭火器
    if (msg.objects.size() < 2 || msg.objects.begin()->type != 10) {
        return;
    }
    std::vector<Point> vpoint;
    for (auto& obj : msg.objects) {
        Point p = { obj.x, obj.y };
        vpoint.push_back(p);
    }
    if (global_->elevator_first_step_ == 1) {

        if (mhindex_ > 4) {
            return;
        }
        Point result = calculatePerpendicularPoint(vpoint[0], vpoint[1], 3.0);
        std::cout << " vpoint[0] x " << vpoint[0].x << " vpoint[0].y " << vpoint[0].y << std::endl;
        std::cout << " vpoint[1] x " << vpoint[1].x << " vpoint[1].y " << vpoint[1].y << std::endl;

        std::cout << " result x " << result.x << " y " << result.y << std::endl;
        // 第一阶段方向
        Point midPoint;
        midPoint.x = (vpoint[0].x + vpoint[1].x) / 2;
        midPoint.y = (vpoint[0].y + vpoint[1].y) / 2;
        // Point directionResult = calculatePerpendicularPoint(vpoint[0], vpoint[1], 0.5);
        std::cout << " directionResult x " << midPoint.x << " y " << midPoint.y << std::endl;

        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = rclcpp::Time();
        pointSta->point.x = result.x;
        pointSta->point.y = result.y;

        geometry_msgs::msg::PointStamped::SharedPtr pointDire = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointDire->header.stamp = rclcpp::Time();
        pointDire->point.x = midPoint.x;
        pointDire->point.y = midPoint.y;
        global_->send_elevator_goal_pose(pointSta, pointDire);
        mhindex_++;
    } else if (global_->elevator_first_step_ == 5) {
        // 判断是否为两个灭火器
        if (mhindex_ > 100) {
            global_->elevator_first_step_ = 2;
            return;
        }
        Point midPoint;
        midPoint.x = (vpoint[0].x + vpoint[1].x) / 2;
        midPoint.y = (vpoint[0].y + vpoint[1].y) / 2;
        std::cout << " vpoint[0] x " << vpoint[0].x << " vpoint[0].y " << vpoint[0].y << std::endl;
        std::cout << " vpoint[1] x " << vpoint[1].x << " vpoint[1].y " << vpoint[1].y << std::endl;
        std::cout << " midPoint x " << midPoint.x << " y " << midPoint.y << std::endl;

        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = rclcpp::Time();
        pointSta->point.x = midPoint.x;
        pointSta->point.y = midPoint.y;

        global_->send_elevator_direction(pointSta);
        mhindex_++;
    } else if (global_->elevator_first_step_ == 2 && elevator_door_condition_ != 0) {
        if (ehindex_ > 4) {
            return;
        }
        Point door1, door2, leftPoint, hightPoint;
        if (vpoint[0].y > 0) {
            leftPoint = vpoint[0];
            hightPoint = vpoint[1];

        } else {
            leftPoint = vpoint[1];
            hightPoint = vpoint[0];
        }

        if (elevator_door_condition_ == 1) {
            door1 = calculateCPoint(hightPoint, leftPoint, 0.50f, 1.7f);
            door2 = calculateCPoint(hightPoint, leftPoint, 0.50f, 0.50f);
        } else if (elevator_door_condition_ == 2) {
            door1 = calculateCPoint(leftPoint, hightPoint, 0.50f, 1.7f);
            door2 = calculateCPoint(leftPoint, hightPoint, 0.50f, 0.50f);
        }
        std::cout << "hightPoint x " << hightPoint.x << " hightPoint.y " << hightPoint.y << std::endl;
        std::cout << " leftPoint x " << leftPoint.x << " leftPoint.y " << leftPoint.y << std::endl;
        std::cout << " door1 x " << door1.x << " y " << door1.y << std::endl;
        std::cout << " door2 x " << door2.x << " y " << door2.y << std::endl;
        std::cout << " elevator_door_condition_ " << elevator_door_condition_ << std::endl;
        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = rclcpp::Time();
        pointSta->point.x = door1.x;
        pointSta->point.y = door1.y;

        geometry_msgs::msg::PointStamped::SharedPtr pointDire = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointDire->header.stamp = rclcpp::Time();
        pointDire->point.x = door2.x;
        pointDire->point.y = door2.y;

        global_->send_elecator_second_goal_pose(pointSta, pointDire);
        ehindex_++;
    }
    // global_->send_elevator_direction(pointDire);
}

void OperationControl::handleOperationScenario(const std::shared_ptr<interface::srv::OperationScenario::Request> request,
    std::shared_ptr<interface::srv::OperationScenario::Response> response)
{
    // auto global_ = service::Global::getInstance();
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    // if (under_way_) {
    //     response->set__code(-1);
    //     return;
    // }
    if (control_statua_[(ControlIndex)request->scene] != ControlStatus::ON) {
        response->set__code(-1);
        return;
    }
    char cmd[256];
    std::cout << " 场景 " << request->scene << std::endl;
    if ((ControlIndex)request->scene == ControlIndex::UP_ELEVATOR) {
        global_->elevator_first_step_ = 1;
        mhindex_ = 0;
        elevator_door_condition_ = 0;
        // global_->direceStatus_ = false;
        sprintf(cmd, "ros2 param set /vlago_ctrl max_speed %f", 0.5);

    } else if ((ControlIndex)request->scene == ControlIndex::DOWN_ELEVATOR) {
        sprintf(cmd, "ros2 param set /vlago_ctrl max_speed %f", 0.5);

        global_->elevator_first_step_ = 6;
        // global_->advance(5.0f);
    } else if ((ControlIndex)request->scene == ControlIndex::GARDEN) {
        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = rclcpp::Time();
        pointSta->point.x = garden_x_;
        pointSta->point.y = garden_y_;
        sprintf(cmd, "ros2 param set /vlago_ctrl max_speed %f", 1.5);
        global_->send_goal_summon(pointSta);
    } else if ((ControlIndex)request->scene == ControlIndex::HOME) {
        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = rclcpp::Time();
        pointSta->point.x = home_x_;
        pointSta->point.y = home_y_;
        sprintf(cmd, "ros2 param set /vlago_ctrl max_speed %f", 1.5);
        global_->send_goal_summon(pointSta);
    }
    system(cmd);
    control_statua_[(ControlIndex)request->scene] = ControlStatus::UNDERWAY;
    interface::msg::VlagoScene msg;
    msg.scene = request->scene;
    msg.onoff = 1;
    sence_pub_->publish(msg);
    response->set__code(0);
}

void OperationControl::handleOperationStatus(const std::shared_ptr<interface::srv::OperationStatus::Request> request,
    std::shared_ptr<interface::srv::OperationStatus::Response> response)
{
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    // auto global_ = service::Global::getInstance();
    response->set__code(0);
    if (global_->elevator_first_step_ == 3) {
        control_statua_[ControlIndex::UP_ELEVATOR] = ControlStatus::ON;
    }
    control_statua_[ControlIndex::GARDEN] = ControlStatus::ON;
    control_statua_[ControlIndex::HOME] = ControlStatus::ON;
    control_statua_[ControlIndex::DOWN_ELEVATOR] = ControlStatus::ON;
    for (auto iter : control_statua_) {
        interface::msg::Operation oper;
        oper.status = (int)iter.second;
        oper.scens = (int)iter.first;
        response->operations.emplace_back(oper);
    }
}

void OperationControl::onDetectorResult(const interface::msg::Objects msg)
{
    // auto global_ = service::Global::getInstance();
    if (global_->elevator_first_step_ > 3) {
        return;
    }
    if (global_->elevator_first_step_ == 2) {

        // 判断是否为电梯开门
        if (msg.objects.size() < 1) {
            return;
        }
        if (ehindex_ > 4) {
            return;
        }
        int index = 0;
        Point point1, point2;
        for (auto& obj : msg.objects) {
            if (obj.type == 21) {
                index++;
                if (obj.y > 0) {
                    elevator_door_condition_ = 1;
                } else {
                    elevator_door_condition_ = 2;
                }
            }
        }
        return;
        // 行驶点
        Point result = calculatePerpendicularPoint(point1, point2, 2.5);
        std::cout << "  第二阶段 result x " << result.x << " y " << result.y
                  << " point1 x " << point1.x << " point1 y " << point1.y
                  << " point2 x " << point2.x << " point2 y " << point2.y << std::endl;
        // 方向点
        Point midPoint;
        midPoint.x = (point1.x + point2.x) / 2;
        midPoint.y = (point1.y + point2.y) / 2;

        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = rclcpp::Time();
        pointSta->point.x = result.x;
        pointSta->point.y = result.y;

        geometry_msgs::msg::PointStamped::SharedPtr pointDire = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointDire->header.stamp = rclcpp::Time();
        pointDire->point.x = midPoint.x;
        pointDire->point.y = midPoint.y;

        global_->send_elecator_second_goal_pose(pointSta, pointDire);
        ehindex_++;
    }

    if (global_->elevator_first_step_ == 3) {
        // return;
        //  判断是否为电梯开门
        if (msg.objects.size() < 1) {
            // std::cout << " elevator_first_step_ = 3 object size : " << msg.objects.size() << std::endl;
            return;
        }

        if (third_index_ > 4) {
            // std::cout << " elevator_first_step_ = 3 third_index_ : " << third_index_ << std::endl;
            return;
        }

        int index = 0;
        Point point1, point2;
        for (auto& obj : msg.objects) {
            if (obj.type == 21) {
                index++;
                point1 = Point { obj.x, obj.y };
                point2 = Point { obj.speedx, obj.speedy };
            }
        }
        if (index == 0) {
            return;
        }

        // 行驶点
        Point result = calculatePerpendicularPoint(point1, point2, 0.5);
        std::cout << "  第三阶段 result x " << result.x << " y " << result.y
                  << " point1 x " << point1.x << " point1 y " << point1.y
                  << " point2 x " << point2.x << " point2 y " << point2.y << std::endl;
        // std::cout << "  第三阶段 result x " << result.x << " y " << result.y << std::endl;
        //  方向点
        Point midPoint;
        midPoint.x = (point1.x + point2.x) / 2;
        midPoint.y = (point1.y + point2.y) / 2;

        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = rclcpp::Time();
        pointSta->point.x = result.x;
        pointSta->point.y = result.y;

        geometry_msgs::msg::PointStamped::SharedPtr pointDire = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointDire->header.stamp = rclcpp::Time();
        pointDire->point.x = midPoint.x;
        pointDire->point.y = midPoint.y;

        global_->send_elecator_third_goal_pose(pointSta, pointDire);
        third_index_++;
    }
}