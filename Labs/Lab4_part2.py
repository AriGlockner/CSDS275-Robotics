# -*- coding: utf-8 -*-
"""
VSCode Editor

@Sai
"""
import numpy as np
import matplotlib.pyplot as plt
import coppeliasim_zmqremoteapi_client as zmq


def get_LiDar_data(sim):
    ''' 
    This function should receive and unpack your sensor reading from CoppeliaSim 
    Fill in the code to get rawData from CoppeliaSim
    '''
    # TODO
    rawData = sim.callScriptFunction('getMeasuredData@/fastHokuyo', sim.scripttype_childscript)
    # Processing sensor reading by calling the getMeasuredData function you wrote in CoppeliaSim, check sim.callScriptFunction().
    # The syntax should be "function_name@object_handle_whose_script_the_function_resides
    reading = np.array(rawData).reshape(-1,3)
    return reading


def get_vision_data(sim, handleName):
    handle = sim.getObject(handleName)
    '''
    You may notice that when simulation is running there is a vision sensor showing the scene via floating view window.
    Fill in the code so that you can get image and resolution info from your vision sensor in CoppeliaSim side.
    Check sim.getVisionSensorImg().
    '''
    image, resolution = sim.getVisionSensorImg(handle)

    return image, resolution


def main():
    # Initialize connection
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    
    # retrieve data from CoppeliaSim
    """
    Get data from CoppeliaSim
    """
    reading = get_LiDar_data(sim)
    img, res = get_vision_data(sim, '/Vision_sensor')
    
    # Some process step for image data
    img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    img = np.fliplr(img)
    plt.imshow(img)
    
    # plot code
    for point in reading:
        plt.plot(point[0], point[1], '.', color='k')
    plt.show()


if __name__ == '__main__':
    main()
