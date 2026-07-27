#include "follow_control.h"

using namespace service::control;

FollowControl::FollowControl()
    : Control("follow")
{
    using namespace std::placeholders;
    auto global = service::Global::getInstance();
    follow_me_ = global->node_->create_service<interface::srv::SceneControls>(sceneFollowmeService, std::bind(&FollowControl::handleFollowMe, this, _1, _2));
    follow_sta_ = global->node_->create_service<interface::srv::FollowStatus>(followControlStatusService, std::bind(&FollowControl::handleFollowStatus, this, _1, _2));
    object_sub_ = global->node_->create_subscription<interface::msg::Objects>(perTracekerTopic, 10, std::bind(&FollowControl::onTrackerResult, this, _1));
    follow_pub_ = global->node_->create_publisher<interface::msg::Nav2FollowMe>(ServiceFollowControlTopic, 30);
    follow_status_pub_ = global->node_->create_publisher<std_msgs::msg::Int32>(followmeStatusTopic, 10);
    scene_control_client_ = global->node_->create_client<interface::srv::SceneControls>(manageFollowMe);
    timer_ = std::make_unique<utils::NonBlockingTimer>(1000, [this]() { this->onTimer(); });
    timer_->start();
}

FollowControl::~FollowControl()
{
    timer_->stop();
}

void FollowControl::handleFollowMe(const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response)
{
    std::cout << " handleFollowMe 0 : " << request->start << std::endl;
    auto global = service::Global::getInstance();
    global->cannel_goal_pose();
    if (request->start) {
        sta_index_ = 0;
        is_start_status = true;
        change_status(0);
        index_ = 0;
    } else {
        sta_index_ = 0;
        is_start_status = false;
        change_status(2);
        objs_ = interface::msg::Objects();
        track_id_ = 0;
    }
    std::cout << " handleFollowMe 1 : " << request->start << std::endl;
    response->set__code(0);
    response->set__message("success");
}

void FollowControl::onTimer()
{
    if (is_start_status) {
        if (sta_index_ > 4) {
            is_start_status = false;
            // std::cout << " onTimer foolow_status_ = 2" << std::endl;
            change_status(2);
            objs_ = interface::msg::Objects();
            track_id_ = 0;
        }
        sta_index_++;
    }
}

void FollowControl::handleFollowStatus(const std::shared_ptr<interface::srv::FollowStatus::Request> request,
    std::shared_ptr<interface::srv::FollowStatus::Response> response)
{
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    response->set__code(0);
    response->status = foolow_status_;
}
// 计算两个点之间的欧氏距离
double FollowControl::distance(const Point& p1, const Point& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 检查一个点是否在矩形框内
bool FollowControl::is_point_in_rect(const Point& p, const Rect& rect)
{
    int left = std::min(rect.top_left.x, rect.bottom_right.x);
    int right = std::max(rect.top_left.x, rect.bottom_right.x);
    int top = std::min(rect.top_left.y, rect.bottom_right.y);
    int bottom = std::max(rect.top_left.y, rect.bottom_right.y);

    return (p.x >= left && p.x <= right && p.y >= top && p.y <= bottom);
}

// 查找在矩形框内距离中心点最近的点
std::pair<int, Point> FollowControl::find_closest_point_in_rect(const std::map<int, Point>& mpoint, const Rect& rect)
{
    // 计算矩形的中心点
    Point center;
    center.x = (rect.top_left.x + rect.bottom_right.x) / 2;
    center.y = (rect.top_left.y + rect.bottom_right.y) / 2;
    // std::cout << "  center.x " << center.x << " center.y " << center.y << std::endl;
    double min_distance = std::numeric_limits<double>::max();
    std::pair<int, Point> closest_point = { -1, { 0.0, 0.0 } }; // 默认无效点

    // 遍历所有的点
    for (const auto& [id, point] : mpoint) {
        // 检查点是否在矩形框内
        if (is_point_in_rect(point, rect)) {
            // 计算到中心点的距离
            std::cout << " ----------point x ----------" << point.x << std::endl;
            double dist = distance(point, center);
            // 如果找到更近的点，更新最近点
            if (dist < min_distance) {
                min_distance = dist;
                closest_point = { id, point };
            }
        }
    }

    return closest_point;
}
void FollowControl::onTrackerResult(const interface::msg::Objects msg)
{
    // std::cout << " is_start_status " << is_start_status << " foolow_status " << foolow_status_ << std::endl;
    if (is_start_status && foolow_status_ != 2) {
        if (index_ > 45) {
            change_status(2);
            is_start_status = false;
            track_id_ = 0;
            std::cout << " onTrackerResult foolow_status_ " << foolow_status_ << std::endl;
        }
        objs_ = msg;
        std::map<int, Point> mpoint;
        for (auto& obj : objs_.objects) {
            Point p = { obj.x, obj.y };
            mpoint[obj.id] = p;
            std::cout << " ++++++++++ obj.id " << obj.id << " x " << p.x << " y " << p.y << std::endl;
        }
        geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
        pointSta->header.stamp = msg.header.stamp;
        auto global = service::Global::getInstance();

        if (track_id_ == 0) {
            std::pair<int, Point> closest_point = find_closest_point_in_rect(mpoint, rect_);
            if (closest_point.first != -1) {
                pointSta->point.x = closest_point.second.x;
                pointSta->point.y = closest_point.second.y;
                change_status(1);
                track_id_ = closest_point.first;
                if (pointSta->point.x > 1.5) {
                    // pointSta->point.x = pointSta->point.x - 2.0;
                    global->send_goal_pose(pointSta);
                }
                std::cout << " track_id_ " << track_id_ << std::endl;
                return;
            }
        }
        if (track_id_ != 0 && ++index_obj_ % 5 == 0 && mpoint.size() > 0) {
            if (mpoint.find(track_id_) != mpoint.end()) {
                if(mpoint[track_id_].x > 3.5){
                    pointSta->point.x = mpoint[track_id_].x - 1.0;
                }
                else {
                    pointSta->point.x = mpoint[track_id_].x;
                }
                pointSta->point.y = mpoint[track_id_].y;
                change_status(1);
                index_ = 0;
                sta_index_ = 0;
                if (pointSta->point.x > 1.5) {
                    // pointSta->point.x = pointSta->point.x - 2.0;
                    // std::cout << " track_id_ " << track_id_ << std::endl;
                    global->send_goal_pose(pointSta);
                }
            } else {
                ++index_;
            }
        }
    }
}


void FollowControl::change_status(int status)
{
    if (foolow_status_ != status) {
        foolow_status_ = status;
	std_msgs::msg::Int32 msg;
        msg.data = foolow_status_;
        follow_status_pub_->publish(msg);
    }
}
