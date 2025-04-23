import sys
import os
import argparse
import time
import multiprocessing

import cv2
import numpy as np
from PIL import Image

import kinova

sys.path.insert(1, 'C:\\Users\\TARS\\Desktop\\dev\\k4a_python\\pyKinectAzure')
import pykinect_azure as pykinect

sys.path.insert(1, '..\\')
import handler_scene_data

sys.path.insert(1, '..\\turntable_control')
from turntable_control import Turntable

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


# ------------------- capturing utilities -------------------

def save_image(image_array, save_path, image_name):
    im = Image.fromarray(image_array)
    im.save(os.path.join(save_path, image_name))


def load_image(path):
    return np.asarray(Image.open(path))


def kinect_capture(device, image_id, color_save_path, depth_save_path):
    kcolor, kdepth = kinectCapture(
        device=device,
        image_id=image_id
    )

    if kcolor is not None:
        save_image(kcolor, save_path=color_save_path)
    else:
        print("kcolor is None")

    if kdepth is not None:
        save_image(kdepth, save_path=depth_save_path)
    else:
        print("kdepth is None")


def kinectCapture(device, save_path=None, image_id=None, image_name=None, convert_bgr_rgb=True):
    num_max_trials = 10
    for idx in range(num_max_trials):
        # other options in "pykinect_azure\k4a\capture.py"
        # Get the images from the Kinect capture
        kinect_capture = device.update()
        Kinect_color_ret, Kinect_color_image = kinect_capture.get_color_image()
        kinect_capture = device.update()
        Kinect_depth_ret, Kinect_depth_image = kinect_capture.get_depth_image()  # ----- uncomment for normal capturing
        if all([Kinect_color_ret, Kinect_depth_ret]):
            if idx > 0:
                print("Success!")
            # if images are good - return == True
            break
        else:
            # if any of the return values are bad, let operator know
            print("Kinect capture #{} color return: {}".format(image_id, Kinect_color_ret))
            print("Kinect capture #{} depth return: {}".format(image_id, Kinect_depth_ret))
            print("Retrying for #%s..." % image_id)

            time.sleep(idx + 1)
    else:
        print("Failed for #%s after %d trials!" % (image_id, num_max_trials))

    if convert_bgr_rgb and Kinect_color_ret and Kinect_color_image is not None:
        Kinect_color_image = cv2.cvtColor(Kinect_color_image, cv2.COLOR_BGR2RGB)

    if save_path is not None and os.path.exists(save_path):
        save_image(Kinect_color_image, os.path.join(save_path, "{}_color.png".format(image_name)))
        save_image(Kinect_depth_image, os.path.join(save_path, "{}_depth.png".format(image_name)))

    return Kinect_color_image, Kinect_depth_image


def robots_simultaneous_action(action_function, action_params=None):
    p_robot1 = multiprocessing.Process(target=action_function, args=(kargs1, action_params,))
    p_robot2 = multiprocessing.Process(target=action_function, args=(kargs2, action_params,))

    p_robot1.start()
    p_robot2.start()

    p_robot1.join()
    p_robot2.join()


def validate_capture(dir_path, num_shells=6):
    bad_ones = []
    extra_ones = []
    # iterate through all shells, check frame counts
    shell_list = os.listdir(dir_path)
    if len(shell_list) < num_shells:
        print("only {} shell directories found: {}".format(len(shell_list), shell_list))
    for shell in shell_list:
        shell_path = os.path.join(dir_path, shell)
        camera_folders = os.listdir(shell_path)
        if len(camera_folders) < 2:
            print("{} only has cameras {}".format(shell, camera_folders))
        for cam in camera_folders:
            image_total = 120 if "shell0" in shell else 104
            num_images_on_disk = len(os.listdir(os.path.join(shell_path, cam)))
            if num_images_on_disk < image_total:
                bad_ones.append([dir, shell, cam, num_images_on_disk])
            if num_images_on_disk > image_total:
                extra_ones.append([dir, shell, cam, num_images_on_disk])

    if len(bad_ones) == 0 and len(extra_ones) == 0:
        print("All frames accounted for")
        return True

    else:
        print("extras:")
        for e in extra_ones:
            print(e)
        print("bads:")
        for b in bad_ones:
            print(b)
        print("extras: {}".format(len(extra_ones)))
        print("bads: {}".format(len(bad_ones)))

        return False


if __name__ == "__main__":
    argument_parser = argparse.ArgumentParser(
        description="",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    argument_parser.add_argument(
        "--root",
        default="capture_path",
        type=str,
        nargs="+",
        help="Specify root directory where all the captures are stored.",
    )
    argument_parser.add_argument(
        "--kinova_ip1",
        default="192.168.1.10",
        type=str,
        help="Specify IP used to connect to the Kinova directly.",
    )
    argument_parser.add_argument(
        "--kinova_ip2",
        default="192.168.1.12",
        type=str,
        help="Specify IP used to connect to the second Kinova directly.",
    )
    argument_parser.add_argument(
        "--dontUseKinect",
        default=False,
        action="store_true",
        help="Default: False. If true, will not use Kinect for capturing.",
    )
    argument_parser.add_argument(
        "--dontUseTurntable",
        default=False,
        action="store_true",
        help="Default: False. If true, will not use turntable for capturing.",
    )
    argument_parser.add_argument(
        "--sceneID",
        default=None,
        type=str,
        help="Specify sceneID that you intend to capture.",
    )
    argument_parser.add_argument(
        "--shellID",
        default=None,
        type=int,
        help="Specify shellID that you intend to capture. [0,1)",
    )
    argument_parser.add_argument(
        "--clutterLevel",
        default=None,
        type=int,
        help="Specify clutterLevel that you intend to capture.",
    )
    argument_parser.add_argument(
        "--dontCapture",
        default=False,
        action="store_true",
        help="Default: False. If true, will only report items in scene.",
    )
    argument_parser.add_argument(
        "--performCalibration",
        default=False,
        action="store_true",
        help="Default: False. If true, will take images to perform stereo camera calibration.",
    )

    args = argument_parser.parse_args()

    useKinect = not args.dontUseKinect
    useTurnTable = not args.dontUseTurntable

    # initialize the data handler
    dhandler = handler_scene_data.DataHandler()

    if not args.performCalibration:
        # set up capture directories
        assert args.sceneID is not None, "specify a sceneID"
        # root = os.path.join("C:\\Users\\TARS\\Desktop\\dev\\hi_lo\\Hi-Lo\\test_captures", args.sceneID)
        root = os.path.join("C:\\Capture")
        if not os.path.exists(root):
            os.mkdir(root)
        root = os.path.join(root, args.sceneID)
        if not os.path.exists(root):
            os.mkdir(root)
        else:
            print("capture directory already exists. quitting...")
            exit()

        # print out some info for user
        print("Beginning data collection for sceneID: {}".format(args.sceneID))
        if args.clutterLevel is not None:
            print("Beginning data collection for clutter level: {}".format(args.clutterLevel))
        print("objects in scene: {}".format(sorted(dhandler.getObjectsForScene(sceneName=args.sceneID))))
        if args.sceneID is not dhandler.allScenes[-1]:
            print("objects in next scene: {}".format(
                sorted(dhandler.getObjectsForScene(sceneName=str(int(args.sceneID) + 1)))))
        else:
            print("final scene!")
        if args.dontCapture:
            exit()

    if useKinect:  # set up Kinect
        kinect_color_keyword = "kinect_color"
        kinect_depth_keyword = "kinect_depth"
        # Initialize the library, if the library is not found, add the library path as argument
        pykinect.initialize_libraries()
        # Modify camera configuration
        device_config = pykinect.default_configuration
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
        # Start device
        kinect1 = pykinect.start_device(device_index=0, config=device_config)
        kinect2 = pykinect.start_device(device_index=1, config=device_config)

    if useTurnTable:
        turntable = Turntable()

    # default args: ip1: 192.168.1.10, username: "admin", password: "admin"
    # default args: ip2: 192.168.1.12, username: "admin", password: "admin"
    kinova_ip1 = args.kinova_ip1
    kinova_ip2 = args.kinova_ip2
    username = "admin"
    password = "admin"
    kargs1 = [kinova_ip1, username, password]
    kargs2 = [kinova_ip2, username, password]

    # Create required services
    movement_success = True

    if args.performCalibration:
        calib_path = "C:\\Users\\TARS\\Desktop\\dev\\Hi_lo\\Hi-Lo\\kinect_kinova_calib"
        calib_path = "C:\\Users\\TARS\\Desktop\\dev\\Hi-Lo\\Hi-Lo\\kinect_kinova_calib"
        cam_group_10_path = os.path.join(calib_path, "group10")
        cam_group_12_path = os.path.join(calib_path, "group12")
        try:
            os.makedirs(calib_path, exist_ok=True)
            os.makedirs(cam_group_10_path, exist_ok=True)
            os.makedirs(cam_group_12_path, exist_ok=True)
            os.makedirs(os.path.join(cam_group_10_path, "kinect_color"), exist_ok=True)
            os.makedirs(os.path.join(cam_group_10_path, "kinect_depth"), exist_ok=True)

        except OSError:
            print("one or more directories could not be created, exiting...")
            exit()

        s = 0
        while True:
            # perform shell captures until we want to stop
            try:
                input("press enter to capture image shell {}".format(s))

                num_images_per_arch = 4
                poses = kinova.generate_poses(
                    arc_radius_inches=dhandler.shellDistances[0],
                    min_angle=20,
                    max_angle=70,
                    num_steps=num_images_per_arch + 1,
                )

                # record a shell
                num_rotations = 4
                for r in range(num_rotations):  # number of rotations
                    # move arm to start positions
                    print("starting arc {} and {}".format(r, r + num_rotations))
                    p1 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs1,))
                    p2 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs2,))
                    p1.start()
                    p2.start()

                    if useTurnTable and r != 0:
                        # move turntable degrees_per_rotation degrees, blocking, except for the first arc
                        turntable.rotate(int(180 / num_rotations), wait=True)

                    p1.join()
                    p2.join()

                    for i in range(num_images_per_arch):
                        # move robot to poses[i]
                        # make sure arm has finished moving and settled (block until movement done + wait 1 second)
                        robots_simultaneous_action(action_function=kinova.robot_to_cartesian_pose,
                                                   action_params=poses[i])

                        time.sleep(1.5)  # make sure stuff settles
                        IMAGES_START_TIME = time.time()
                        robot1_arc_idx = r
                        robot2_arc_idx = int(r + num_rotations)

                        capture_threads = []
                        image_index = s

                        # Capture all together
                        for pp in capture_threads:
                            pp.start()

                        if useKinect:  # capture Kinect image if useKinect
                            # setup kinect1 camera
                            kinect10_color_save_path = os.path.join(cam_group_10_path,
                                                                    "kinect_color",
                                                                    "{}.png".format(image_index))
                            kinect10_depth_save_path = os.path.join(cam_group_10_path,
                                                                    "kinect_depth",
                                                                    "{}.png".format(image_index))
                            kinect_capture(device=kinect1,
                                           image_id="kin1",
                                           color_save_path=kinect10_color_save_path,
                                           depth_save_path=kinect10_depth_save_path)

                            kinect12_color_save_path = os.path.join(cam_group_12_path,
                                                                    "kinect_color",
                                                                    "{}.png".format(image_index))
                            kinect12_depth_save_path = os.path.join(cam_group_12_path,
                                                                    "kinect_depth",
                                                                    "{}.png".format(image_index))

                            kinect_capture(device=kinect2,
                                           image_id="kin2",
                                           color_save_path=kinect12_color_save_path,
                                           depth_save_path=kinect12_depth_save_path)

                        # Wait for all captures to be done
                        for pp in capture_threads:
                            pp.join()

                        s += 1

                # Reset
                p1 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs1,))
                p2 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs2,))
                p1.start()
                p2.start()

                if useTurnTable:
                    # move back to start position
                    # 45 + 180
                    turntable.rotate(225, wait=True)

                p1.join()
                p2.join()

            except KeyboardInterrupt:
                print("\nexiting...")
                break
        kinect1.stop_cameras()
        kinect2.stop_cameras()

        exit()

    else:  # perform cluttered scene capture
        TOTAL_START_TIME = time.time()
        try:
            if kinova.get_robot_y(kargs1) < 0.2:
                print("Moving robot 10 (left) to the home position...")
                kinova.robot_to_home_position(kargs1)

            if kinova.get_robot_y(kargs2) < 0.2:
                print("Moving robot 12 (right) to the home position...")
                kinova.robot_to_home_position(kargs2)

            # --- timing stuff ---
            arcTimes = []
            shellTimes = []
            imageTimes = []
            clutterTimes = []
            # ---------- BEGIN CAPTURE PROCEDURE ----------
            degrees_per_rotation = dhandler.numDegreesPerRotation
            degrees_per_arch_image = dhandler.numDegreesPerImageOnArc
            num_rotations = dhandler.numRotationsForAScene
            num_images_per_arch = dhandler.numImageSetsPerArc
            num_closeness_levels = dhandler.numClutterLevelsPerScene

            c = 0
            bad_found = False
            while c < num_closeness_levels:
                if args.clutterLevel is not None and c != args.clutterLevel:
                    # If clutterLevel is passed, and we aren't on that clutterLevel, continue to next iteration
                    c += 1
                    continue

                TOTAL_START_TIME = time.time()
                CLUTTER_START_TIME = time.time()

                if bad_found:
                    print("Redoing closeness level %d...", c)

                else:
                    print("ensure that objects are arranged for closeness: {}".format(c))
                    print(
                        "where # 0 has objects with higher closeness level (smallest circle), 1 has lower closeness level, "
                        "2 is sparse (anywhere on turntable)")
                    input("press enter when objects are set up and ready for a new shell recording")

                clutterTimes.append(time.time() - CLUTTER_START_TIME)

                for s in range(dhandler.numShellsPerScene):
                    if args.shellID is not None and s != args.shellID:
                        # if shellID is passed and we aren't on that shellID, continue to next iteration
                        continue
                    SHELL_START_TIME = time.time()
                    # set up capture directories
                    shell_name = dhandler.getShellFullName(sceneName=args.sceneID, shellIndex=s, clutterIndex=c)
                    if os.path.exists(os.path.join(root, shell_name)):
                        # if the shell has already been recorded, skip this (don't overwrite!)
                        print("directory for shell {} already exists. skipping...".format(shell_name))
                        continue
                    os.mkdir(os.path.join(root, shell_name))
                    kinect_depth_path = os.path.join(root, shell_name, kinect_depth_keyword)
                    kinect_color_path = os.path.join(root, shell_name, kinect_color_keyword)
                    os.mkdir(kinect_depth_path)
                    os.mkdir(kinect_color_path)
                    # plan arc waypoints
                    if s == 0:
                        num_images_per_arch = dhandler.numImageSetsPerArc
                        poses = kinova.generate_poses(
                            arc_radius_inches=dhandler.shellDistances[s],
                            min_angle=dhandler.arcStartDegree,
                            max_angle=dhandler.arcEndDegree,
                            num_steps=num_images_per_arch,
                        )

                    else:  # need to skip the top 10 degrees (2 images) because of arm motion limits
                        num_images_per_arch = dhandler.numImageSetsPerArc - 2
                        poses = kinova.generate_poses(
                            arc_radius_inches=dhandler.shellDistances[s],
                            min_angle=dhandler.arcStartDegree,
                            max_angle=dhandler.arcEndDegree - 10,
                            num_steps=num_images_per_arch,
                        )

                    # record a shell
                    for r in range(int(num_rotations / 2)):
                        # move arm to start position(s)
                        print("moving arm to start position on arc {} for shell {}".format(r, s))
                        p1 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs1,))
                        p2 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs2,))
                        p1.start()
                        p2.start()

                        if useTurnTable and r != 0:
                            # move turntable degrees_per_rotation degrees, blocking, except for the first arc
                            # print("rotating turntable")
                            turntable.rotate(degrees_per_rotation, wait=True)

                        p1.join()
                        p2.join()

                        ARC_START_TIME = time.time()
                        for i in range(num_images_per_arch):
                            # move robot to poses[i]
                            # make sure arm has finished moving and settled (block until movement done + wait 1 second)
                            robots_simultaneous_action(action_function=kinova.robot_to_cartesian_pose,
                                                       action_params=poses[i])

                            time.sleep(1.5)  # make sure stuff settles
                            IMAGES_START_TIME = time.time()
                            robot1_arc_idx = r
                            robot2_arc_idx = int(r + num_rotations / 2)

                            # -----multithread image capture and saving-----
                            capture_threads = []

                            # Capture all together
                            for pp in capture_threads:
                                pp.start()

                            if useKinect:  # capture Kinect image if useKinect
                                # setup kinect1 camera
                                kinect1_color_save_path = os.path.join(kinect_color_path,
                                                                       "kinect_color_arc{}_image{}.png".format(
                                                                           robot1_arc_idx, i))
                                kinect1_depth_save_path = os.path.join(kinect_depth_path,
                                                                       "kinect_depth_arc{}_image{}.png".format(
                                                                           robot1_arc_idx, i))

                                kinect_capture(device=kinect1,
                                               image_id="%s-%s" % (robot1_arc_idx, i),
                                               color_save_path=kinect1_color_save_path,
                                               depth_save_path=kinect1_depth_save_path)

                                # setup kinect2 camera
                                # image set 2 is 180 degrees offset from set 1
                                kinect2_color_save_path = os.path.join(kinect_color_path,
                                                                       "kinect_color_arc{}_image{}.png".format(
                                                                           robot2_arc_idx, i))
                                kinect2_depth_save_path = os.path.join(kinect_depth_path,
                                                                       "kinect_depth_arc{}_image{}.png".format(
                                                                           robot2_arc_idx, i))

                                kinect_capture(device=kinect2,
                                               image_id="%s-%s" % (robot2_arc_idx, i),
                                               color_save_path=kinect2_color_save_path,
                                               depth_save_path=kinect2_depth_save_path)

                            # Wait for all captures to be done
                            for pp in capture_threads:
                                pp.join()

                            # make sure captures have both completed (and saved?)
                            imageTimes.append(time.time() - IMAGES_START_TIME)

                        arcTimes.append(time.time() - ARC_START_TIME)

                    # Reset
                    p1 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs1,))
                    p2 = multiprocessing.Process(target=kinova.robot_to_starting_position, args=(kargs2,))
                    p1.start()
                    p2.start()

                    if useTurnTable:
                        # move back to start position
                        # 45 + 180
                        turntable.rotate(225, wait=True)

                    p1.join()
                    p2.join()

                    shellTimes.append(time.time() - SHELL_START_TIME)

                # Validate all captures and redo a clutter if any bad capture is found.
                if validate_capture(root, num_shells=c * dhandler.numShellsPerScene):
                    bad_found = False

                c += 1

            print("in total, this capture took {}".format(time.time() - TOTAL_START_TIME))
            print("of {} shellTimes, the average time was {}".format(len(shellTimes), np.average(shellTimes)))
            print("of {} arcTimes, the average time was {}".format(len(arcTimes), np.average(arcTimes)))
            print("of {} imageTimes, the average time was {}".format(len(imageTimes), np.average(imageTimes)))
            print("of {} clutterTimes, the average time was {}".format(len(clutterTimes), np.average(clutterTimes)))
            print("clutter arrangement times: {}".format(clutterTimes))
            kinect1.stop_cameras()
            kinect2.stop_cameras()

        except KeyboardInterrupt:
            print("quitting...")
            kinect1.stop_cameras()
            kinect2.stop_cameras()
            print("in total, this capture took {}".format(time.time() - TOTAL_START_TIME))
            exit()
