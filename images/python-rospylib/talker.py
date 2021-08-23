# Example from https://roslibpy.readthedocs.io/en/latest/examples.html
import roslibpy
import time

# Host needs to be the ip of ros-master on the ros-network
client = roslibpy.Ros(host='172.31.0.2', port=9090)
client.run()

talker = roslibpy.Topic(client, '/chatter', 'std_msgs/String')

while client.is_connected:
    talker.publish(roslibpy.Message({ 'data': 'Hello World!' }))
    print('Sending message...')
    time.sleep(1)

talker.unadvertise()
client.terminate()