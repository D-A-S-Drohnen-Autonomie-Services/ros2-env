#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"
using std::placeholders::_1;

class AddressBookSubscriber : public rclcpp::Node
{
public:
    AddressBookSubscriber()
            : Node("address_book_subscriber")
    {
        subscription_ = this->create_subscription<more_interfaces::msg::AddressBook>(
                "address_book", 10, std::bind(&AddressBookSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const more_interfaces::msg::AddressBook & msg) const
    {
        std::string message =
                msg.first_name +
                msg.last_name;
        RCLCPP_INFO(this->get_logger(), message.c_str());
        RCLCPP_INFO(this->get_logger(), msg.gender ? "FEMALE" : "MALE");
    }
    rclcpp::Subscription<more_interfaces::msg::AddressBook>::SharedPtr subscription_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressBookSubscriber>());
    rclcpp::shutdown();
    return 0;
}