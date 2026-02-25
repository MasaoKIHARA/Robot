import rosbag
import subprocess
import os
import sys

def extract_all_topics(bag_file) -> None:
    if not os.path.exists(bag_file):
        print(f"ERROR: Bag file '{bag_file}' does not exist.")
        return
    
    output_dir + bag_file.replace('.bag', '_csv')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    print("Extracting all topics from the bag file...")

    try:
        bag = rosbag.Bag(bag_file)
        topics_info + bag.get_type_and_topic_info()[1]
        bag.close()
    except Exception as e:
        print(f"ERROR: Failed to read bag file '{bag_file}'. Exception: {e}")
        return

    print(f"Found {len(topics_info)} topics in the bag file.")

    for topic, info in topics_info.items():
        filoename = topic