/***************************************************************************************************
 * Copyright Five AI 2021.
 * All rights reserved.
 ***************************************************************************************************/

#include "threading/Utils.h"
#include "ros/shm_engine.h"
#include "threading/ShmQueue.h"
#include "util/ShmUniquePtr.h"
#include "msgs/ShmSharedPtr.h"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <std_msgs/Int32.h>

#include <gtest/gtest.h>

#include <boost_1.65.0/boost/process.hpp>
#include <boost/make_unique.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>

#include <chrono>

int globalArgc = -1;
char** globalArgv = nullptr;

namespace bp = boost::process;

class Shm : public testing::Test
{
public:
    Shm() :
        m_group{},
        m_rosMaster{},
        m_launchRosMaster{extractLaunchRosMasterFromCli()},
        m_children{},
        m_nodeHandle{},
        m_nodeName{extractNameFromCli()}
    {}

protected:
    using RosCallback = boost::function<void(fiveai::std_msgs::shm::SharedPtrConstShmImage)>;
    using SharedPtrConstShmImage = fiveai::std_msgs::shm::SharedPtrConstShmImage;

    void SetUp() override
    {
        if (isMain())
        {
            if (m_launchRosMaster)
            {
                ROS_INFO_STREAM(prefix() << "spawning rosmaster");
                bp::child c("/bin/bash", "-c", "source /opt/ros/kinetic/setup.sh && rosmaster --core",
                            bp::std_out > stdout, bp::std_err > stderr, bp::std_in < stdin,
                            boost::this_process::environment(), m_group);
                m_rosMaster = std::move(c);
            }
        }
    }

    void TearDown() override
    {
        if (isMain())
        {
            std::cout << prefix() << "terminating and waiting for roscore " << std::endl;
            m_rosMaster.terminate();
            m_rosMaster.wait();
            std::cout << prefix() << "roscore terminated" << std::endl;
        }
    }

    static std::string getTestName()
    {
        return testing::UnitTest::GetInstance()->current_test_info()->name();
    }

    static std::string getTestCaseName()
    {
        return testing::UnitTest::GetInstance()->current_test_case()->name();
    }

    static std::string getFullName()
    {
        return getTestCaseName() + "_" + getTestName();
    }

    static std::string getThisExecutable()
    {
        return globalArgv[0];
    }

    bool startRos()
    {
        ros::init(globalArgc, globalArgv, m_nodeName, ros::init_options::NoRosout);
        m_nodeHandle = boost::make_unique<ros::NodeHandle>();

        return true;
    }

    bool shutdownRos()
    {
        ros::shutdown();
        m_nodeHandle.reset();
        ros::waitForShutdown();
        return true;
    }

    struct Ros
    {
        Ros(Shm& parent) :
            parent{parent}
        {
            EXPECT_TRUE(parent.startRos());
        }

        ~Ros()
        {
            EXPECT_TRUE(parent.shutdownRos());
        }

        Shm& parent;
    };

    bool timedWhile(std::function<bool()> condition,
                    std::function<void()> body,
                    const std::chrono::milliseconds timeout)
    {
        const auto start = std::chrono::steady_clock::now();
        while (condition())
        {
            body();

            std::this_thread::sleep_for(std::chrono::milliseconds{2});
            if (std::chrono::steady_clock::now() - start > timeout)
            {
                return false;
            }
        }
        return true;
    }

    static std::string extractNameFromCli()
    {
        using namespace boost::algorithm;
        const auto keyword = "--name=";
        for (int i = 0; i < globalArgc; ++i)
        {
            if (starts_with(globalArgv[i], keyword))
            {
                std::vector<std::string> tokens;
                split(tokens, globalArgv[i], is_any_of("="), token_compress_on);
                return tokens.at(1);
            }
        }
        return "main";
    }

    static bool extractLaunchRosMasterFromCli()
    {
        const auto keyword = "--norosmaster";
        for (int i = 0; i < globalArgc; ++i)
        {
            if (boost::algorithm::starts_with(globalArgv[i], keyword))
            {
                return false;
            }
        }
        return true;
    }

    bool isMain() const
    {
        return m_nodeName == "main";
    }

    struct SpawnProcess
    {
        SpawnProcess(Shm& parent, const std::string& name) :
            parent{parent},
            child{parent.startChild(name)},
            name{name}
        {
        }

        ~SpawnProcess()
        {
            ROS_INFO_STREAM(parent.prefix() << "is waiting for the child process " << name << " to terminate");
            child.wait();
            EXPECT_EQ(child.exit_code(), 0);
        }

        Shm& parent;
        bp::child& child;
        const std::string name;
    };

    bp::child& startChild(const std::string& childName)
    {
        const auto cmd = getThisExecutable() +
                         " --gtest_filter=" + getTestCaseName() + "." + getTestName() +
                         " --name=" + childName;
         auto c = boost::make_unique<bp::child>(cmd,
                                                bp::std_out > stdout, bp::std_err > stderr,
                                                bp::std_in < stdin, boost::this_process::environment(),
                                                m_group);
        m_children.push_back(std::move(c));

        return *m_children.back();
    }

    std::string prefix() const
    {
        return "------" + m_nodeName + ": ";
    }

    bp::group m_group;
    bp::child m_rosMaster;
    bool      m_launchRosMaster;
    std::vector<std::unique_ptr<bp::child>> m_children;

    std::unique_ptr<ros::NodeHandle> m_nodeHandle;
    std::string m_nodeName;

};

/*
 * pub ---intra---> sub
 *
 * Publisher and subscriber live in the same OS process and communicate through SHM.
*/
TEST_F(Shm, Intra1pub1sub1msg)
{
    Ros engage{*this};

    bool imageReceived = false;
    RosCallback callback = [&imageReceived](SharedPtrConstShmImage img)
    {
        EXPECT_EQ(img->header.seq, 12345u);
        imageReceived = true;
    };

    ros::Publisher pub  = m_nodeHandle->advertise<SharedPtrConstShmImage>("/image", 5);
    ros::Subscriber sub = m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback);

    auto& eng = ros::ShmEngine::instance();

    ros::ShmEngine::SharedPtrImage img = eng.allocateSharedImage({1024, 2048});
    img->header.seq = 12345;

    pub.publish(img);

    constexpr std::chrono::milliseconds timeout{5000};
    auto result = timedWhile
    (
        [&imageReceived]{return ! imageReceived;},
        []{ros::spinOnce();},
        timeout
    );

    EXPECT_TRUE(result) << "The shm subscriber has not received the image "
                        << "within " << timeout.count() << "ms has timed out";
}

/*
 *  pub ---inter---> sub
 *
 * Publisher and subscriber live in different OS processes and communicate through SHM.
*/
TEST_F(Shm, Inter1pub1sub1msg)
{
    const std::string publisherChildName = getFullName() + "_pub";
    const std::string subscriberChildName = getFullName() + "_sub";

    if (m_nodeName == publisherChildName)
    {
        Ros engage{*this};

        ros::Publisher pub  = m_nodeHandle->advertise<SharedPtrConstShmImage>("/image", 5);
        auto& eng = ros::ShmEngine::instance();

        ros::ShmEngine::SharedPtrImage img = eng.allocateSharedImage({1024, 2048});
        img->header.seq = 12345;

        constexpr std::chrono::milliseconds timeout{3000};
        ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for subscribers to appear...");
        auto result = timedWhile
        (
            [&pub]{return pub.getNumSubscribers() == 0;},
            []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
            timeout
        );
        ASSERT_TRUE(result) << prefix() << "no subscriber appeared";

        ROS_INFO_STREAM(prefix() << "publishing one image");
        pub.publish(img);
    }
    else if (m_nodeName == subscriberChildName)
    {
        Ros engage{*this};
        SpawnProcess pub{*this, publisherChildName};

        bool imageReceived = false;
        RosCallback callback = [&imageReceived](SharedPtrConstShmImage img)
        {
            EXPECT_EQ(img->header.seq, 12345u);
            imageReceived = true;
        };

        ros::Subscriber sub = m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback );

        constexpr std::chrono::milliseconds timeout{3000};

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for publishers to appear...");
            auto result = timedWhile
            (
                [&sub]{return sub.getNumPublishers()== 0;},
                []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
                timeout
            );
            ASSERT_TRUE(result) << prefix() << "no publisher appeared";
        }

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for the image to be published...");
            auto result = timedWhile
            (
                [&imageReceived]{return ! imageReceived;},
                []{ros::spinOnce();},
                timeout
            );

            EXPECT_TRUE(result) << "The shm subscriber has not received the image "
                                << "within " << timeout.count() << "ms has timed out";
        }
    }
    else
    {
        SpawnProcess pub{*this, subscriberChildName};
    }
}

/*
 *  pub1 ---inter---> sub
 *  ...              /
 *  pub20---inter---/
 *
 * Publishers and the subscriber live in different OS processes and communicate through SHM.
*/
TEST_F(Shm, DISABLED_Inter20pub1sub1msg)
{
    std::vector<std::string> publisherChildrenName;
    constexpr int PUBLISHER_COUNT = 20;
    for (int i = 0; i < PUBLISHER_COUNT; ++i)
    {
        publisherChildrenName.push_back(getFullName() + "_pub" + std::to_string(i));
    }
    const std::string subscriberChildName = getFullName() + "_sub";

    auto predicate = [this](const std::string& name){return m_nodeName == name;};
    const auto isPublisherChild = std::any_of(publisherChildrenName.cbegin(),
                                              publisherChildrenName.cend(),
                                              predicate);
    if (isPublisherChild)
    {
        Ros engage{*this};

        ros::Publisher pub  = m_nodeHandle->advertise<SharedPtrConstShmImage>("/image", 5);
        auto& eng = ros::ShmEngine::instance();

        ros::ShmEngine::SharedPtrImage img = eng.allocateSharedImage({1024, 2048});
        img->header.seq = 12345;

        constexpr std::chrono::milliseconds timeout{3000};
        ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for subscribers to appear...");
        auto result = timedWhile
        (
            [&pub]{return pub.getNumSubscribers() == 0;},
            []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
            timeout
        );
        ASSERT_TRUE(result) << prefix() << "no subscriber appeared";

        ROS_INFO_STREAM(prefix() << "publishing one image");
        pub.publish(img);
    }
    else if (m_nodeName == subscriberChildName)
    {
        Ros engage{*this};
        std::vector<std::unique_ptr<SpawnProcess>> pubs;
        for (const auto& name : publisherChildrenName)
        {
            pubs.push_back(boost::make_unique<SpawnProcess>(*this, name));
        }

        int imageReceivedCount = 0;
        RosCallback callback = [&imageReceivedCount](SharedPtrConstShmImage img)
        {
            EXPECT_EQ(img->header.seq, 12345u);
            ++imageReceivedCount;
        };

        ros::Subscriber sub = m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback);

        constexpr std::chrono::milliseconds timeout{10000};

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for publishers to appear...");
            auto result = timedWhile
            (
                [&sub]{return sub.getNumPublishers()== 0;},
                []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
                timeout
            );
            ASSERT_TRUE(result) << prefix() << "no publisher appeared";
        }

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for the image to be published...");
            auto result = timedWhile
            (
                [&imageReceivedCount]{return imageReceivedCount != PUBLISHER_COUNT;},
                []{ros::spinOnce();},
                timeout
            );

            EXPECT_TRUE(result) << "The shm subscriber has not received the image "
                                << "within " << timeout.count() << "ms has timed out";
        }
    }
    else
    {
        SpawnProcess pub{*this, subscriberChildName};
    }
}

/*
 *  pub ---inter---> sub1
 *     \             ....
 *      \--inter---> sub20
 *
 * The publisher and the subscribers live in different OS processes and communicate through SHM.
*/
TEST_F(Shm, Inter1pub20sub1msg)
{
    const std::string publisherChildName = getFullName() + "_pub";
    std::vector<std::string> subscriberChildrenName;
    constexpr int SUBSCRIBER_COUNT = 20;
    for (int i = 0; i < SUBSCRIBER_COUNT; ++i)
    {
        subscriberChildrenName.push_back(getFullName() + "_sub" + std::to_string(i));
    }

    auto predicate = [this](const std::string& name){return m_nodeName == name;};
    const auto isSubscriberChild = std::any_of(subscriberChildrenName.cbegin(),
                                               subscriberChildrenName.cend(),
                                               predicate);
    if (m_nodeName == publisherChildName)
    {
        Ros engage{*this};

        std::vector<std::unique_ptr<SpawnProcess>> subs;
        for (const auto& name : subscriberChildrenName)
        {
            subs.push_back(boost::make_unique<SpawnProcess>(*this, name));
        }

        ros::Publisher pub  = m_nodeHandle->advertise<SharedPtrConstShmImage>("/image", 5);
        auto& eng = ros::ShmEngine::instance();

        ros::ShmEngine::SharedPtrImage img = eng.allocateSharedImage({1024, 2048});
        img->header.seq = 12345;

        constexpr std::chrono::milliseconds timeout{5000};
        ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for subscribers to appear...");
        auto result = timedWhile
        (
            [&pub]{return pub.getNumSubscribers() != SUBSCRIBER_COUNT;},
            []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
            timeout
        );
        ASSERT_TRUE(result) << prefix() << "no subscriber appeared";

        ROS_INFO_STREAM(prefix() << "publishing one image");
        pub.publish(img);
    }
    else if (isSubscriberChild)
    {
        Ros engage{*this};

        bool imageReceived = false;
        RosCallback callback = [&imageReceived](SharedPtrConstShmImage img)
        {
            EXPECT_EQ(img->header.seq, 12345u);
            imageReceived = true;
        };

        ros::Subscriber sub = m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback );

        constexpr std::chrono::milliseconds timeout{3000};

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for publishers to appear...");
            auto result = timedWhile
            (
                [&sub]{return sub.getNumPublishers()== 0;},
                []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
                timeout
            );
            ASSERT_TRUE(result) << prefix() << "no publisher appeared";
        }

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for the image to be published...");
            auto result = timedWhile
            (
                [&imageReceived]{return ! imageReceived;},
                []{ros::spinOnce();},
                timeout
            );

            EXPECT_TRUE(result) << "The shm subscriber has not received the image "
                                << "within " << timeout.count() << "ms has timed out";
        }
    }
    else
    {
        SpawnProcess pub{*this, publisherChildName};
    }
}

/*
 *  pub ---inter---> sub1
 *     \
 *      \--intra---> sub2
 *
 * Publisher and subscriber 2 live in same OS process, whilst the publisher
 * and the subscriber sub2 live in different OS processes. Communication is
 * achieved through SHM in either case.
*/
TEST_F(Shm, Inter1pub1sub_Intra1pub1sub_1msg)
{
    const std::string publisherChildName = getFullName() + "_pub";
    const std::string subscriberChildName = getFullName() + "_sub";

    if (m_nodeName == publisherChildName)
    {
        Ros engage{*this};

        ros::Publisher pub  = m_nodeHandle->advertise<SharedPtrConstShmImage>("/image", 5);
        auto& eng = ros::ShmEngine::instance();

        ros::ShmEngine::SharedPtrImage img = eng.allocateSharedImage({1024, 2048});
        img->header.seq = 12345;

        bool imageReceived = false;
        RosCallback callback = [&imageReceived](SharedPtrConstShmImage img)
        {
            EXPECT_EQ(img->header.seq, 12345u);
            imageReceived = true;
        };

        ros::Subscriber intraSub = m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback);

        constexpr std::chrono::milliseconds timeout{3000};
        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for subscribers to appear...");
            auto result = timedWhile
            (
                [&pub]{return pub.getNumSubscribers() != 2;},
                []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
                timeout
            );
            ASSERT_TRUE(result) << prefix() << "no subscriber appeared";
        }

        ROS_INFO_STREAM(prefix() << "publishing one image");
        pub.publish(img);

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for the image to be published...");
            auto result = timedWhile
            (
                [&imageReceived]{return ! imageReceived;},
                []{ros::spinOnce();},
                timeout
            );

            EXPECT_TRUE(result) << "The shm intra subscriber has not received the image "
                                << "within " << timeout.count() << "ms has timed out";
        }
    }
    else if (m_nodeName == subscriberChildName)
    {
        Ros engage{*this};
        SpawnProcess pub{*this, publisherChildName};

        bool imageReceived = false;
        RosCallback callback = [&imageReceived](SharedPtrConstShmImage img)
        {
            EXPECT_EQ(img->header.seq, 12345u);
            imageReceived = true;
        };

        ros::Subscriber sub = m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback );

        constexpr std::chrono::milliseconds timeout{3000};

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for publishers to appear...");
            auto result = timedWhile
            (
                [&sub]{return sub.getNumPublishers()== 0;},
                []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
                timeout
            );
            ASSERT_TRUE(result) << prefix() << "no publisher appeared";
        }

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for the image to be published...");
            auto result = timedWhile
            (
                [&imageReceived]{return ! imageReceived;},
                []{ros::spinOnce();},
                timeout
            );

            EXPECT_TRUE(result) << "The shm inter subscriber has not received the image "
                                << "within " << timeout.count() << "ms has timed out";
        }
    }
    else
    {
        SpawnProcess pub{*this, subscriberChildName};
    }
}

/*
 *  pub ---inter---> sub1
 *    \
 *     \---intra---> sub2
 *      \             ....
 *       \--intra---> sub22
 *
 * Publisher and subscriber 1 live in diferent OS processes, whilst the publisher
 * and the subscribers [sub2, sub22] live in different OS processes. Communication is
 * achieved through SHM in all cases.
*/
TEST_F(Shm, Inter1pub1sub_Intra1pub20sub_1msg)
{
    const std::string publisherChildName = getFullName() + "_pub";
    const std::string subscriberChildName = getFullName() + "_sub";
    constexpr int INTRA_SUBSCRIBER_COUNT = 20;

    if (m_nodeName == publisherChildName)
    {
        Ros engage{*this};

        ros::Publisher pub  = m_nodeHandle->advertise<SharedPtrConstShmImage>("/image", 5);
        auto& eng = ros::ShmEngine::instance();

        ros::ShmEngine::SharedPtrImage img = eng.allocateSharedImage({1024, 2048});
        img->header.seq = 12345;

        std::atomic<int> imageCount{0};
        RosCallback callback = [&imageCount](SharedPtrConstShmImage img)
        {
            EXPECT_EQ(img->header.seq, 12345u);
            ++imageCount;
        };

        ros::Subscriber intraSub[INTRA_SUBSCRIBER_COUNT];
        std::generate(std::begin(intraSub), std::end(intraSub),
                       [this, callback]()
                       {
                            return m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback);
                       });

        constexpr std::chrono::milliseconds timeout{3000};
        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for subscribers to appear...");
            auto result = timedWhile
            (
                [&pub]{return pub.getNumSubscribers() != 2;},
                []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
                timeout
            );
            ASSERT_TRUE(result) << prefix() << "no subscriber appeared";
        }

        ROS_INFO_STREAM(prefix() << "publishing one image");
        pub.publish(img);

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for the image to be published...");
            auto result = timedWhile
            (
                [&imageCount]{return imageCount != INTRA_SUBSCRIBER_COUNT;},
                []{ros::spinOnce();},
                timeout
            );

            EXPECT_TRUE(result) << "The shm intra subscriber has not received the image "
                                << "within " << timeout.count() << "ms has timed out";
        }

        EXPECT_EQ(imageCount, INTRA_SUBSCRIBER_COUNT);
    }
    else if (m_nodeName == subscriberChildName)
    {
        Ros engage{*this};
        SpawnProcess pub{*this, publisherChildName};

        bool imageReceived = false;
        RosCallback callback = [&imageReceived](SharedPtrConstShmImage img)
        {
            EXPECT_EQ(img->header.seq, 12345u);
            imageReceived = true;
        };

        ros::Subscriber sub = m_nodeHandle->subscribe<SharedPtrConstShmImage>("/image", 5, callback );

        constexpr std::chrono::milliseconds timeout{3000};

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for publishers to appear...");
            auto result = timedWhile
            (
                [&sub]{return sub.getNumPublishers()== 0;},
                []{std::this_thread::sleep_for(std::chrono::milliseconds(20));},
                timeout
            );
            ASSERT_TRUE(result) << prefix() << "no publisher appeared";
        }

        {
            ROS_INFO_STREAM(prefix() << "waiting " << timeout.count() << "ms for the image to be published...");
            auto result = timedWhile
            (
                [&imageReceived]{return ! imageReceived;},
                []{ros::spinOnce();},
                timeout
            );

            EXPECT_TRUE(result) << "The shm inter subscriber has not received the image "
                                << "within " << timeout.count() << "ms has timed out";
        }
    }
    else
    {
        SpawnProcess pub{*this, subscriberChildName};
    }
}


TEST_F(Shm, Ros_shutdown)
{
    constexpr int COUNT = 20;

    for ( int i = 0; i < COUNT; ++i )
    {
        Ros engage{*this};

        boost::function<void(std_msgs::Int32)> callback = [](std_msgs::Int32){};
        ros::Subscriber sub = m_nodeHandle->subscribe<std_msgs::Int32>("test", 1, callback);
        ASSERT_TRUE(sub);

        ros::Publisher pub = m_nodeHandle->advertise<std_msgs::Int32>("test2", 1);
        ASSERT_TRUE(pub);

        ros::shutdown();
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    // make the assignment here to make sure gtest has removed its arguments from the list
    globalArgc = argc;
    globalArgv = argv;

    fiveai::threading::baptizeThisThread("main");

    const auto ret = RUN_ALL_TESTS();
    return ret;
}
