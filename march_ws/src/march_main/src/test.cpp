int main(int argc, char **argv)
{

    //initialise rosnode with given name
    ros::init(argc, argv, "data_logger");

    //create nodeHandle object which represents the current ros node
    ros::NodeHandle nh;

    init_parameters();

    string filePath = init_data_logger_storage(argv[0]);

    for(int i = 0; i < *get_number_of_joints(); i++)
    {
        //create publishers w/ datatype, topic name and queue size
        string postfix = (*get_list_of_joints())[i];

        jointSubList.push_back(
                new ros::Subscriber(
                        nh.subscribe(
                                "ECto" + postfix,
                                1000,
                                &log_joint_data
                        )
                )
        );

        targetPosSubList.push_back(
                new ros::Subscriber(
                        nh.subscribe(
                                postfix + "TargetPositionIU",
                                1000,
                                &log_target_pos_iu
                        )
                )
        );

    };

    for(int i = 0; i < *get_number_of_GES(); i++)
    {
        //create publishers w/ datatype, topic name and queue size
        string postfix = (*get_list_of_GES())[i];

        gesSubList.push_back(
                new ros::Subscriber(
                        nh.subscribe(
                                "ECto" + postfix,
                                1000,
                                &log_ges_data
                        )
                )
        );

    };

    rateSubPtr = new ros::Subscriber(
            nh.subscribe(
                    "daLoRate",
                    1,
                    &set_counter_rate
            )
    );


    ros::Rate rate(*get_ethercat_frequency());
    counterRate = 200;

    while(ros::ok())
    {
        ros::spinOnce();

        counter = (counter + 1) % counterRate;

        if(counter == 0)
        {
            log_data_to_file(filePath);
        }

        rate.sleep();
    }

}