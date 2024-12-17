# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import json
# import pyttsx3

# class ObjectSubscriber(Node):
#     def __init__(self):
#         super().__init__('object_subscriber')
#         self.subscription = self.create_subscription(String, '/object_data', self.listener_callback, 10)
#         self.detected_objects = set()
#         self.tts_engine = pyttsx3.init()
#         self.get_logger().info("Node initialized and ready to process data")

#     def speak(self, message):
#         self.get_logger().info(f'Speaking: {message}')
#         self.tts_engine.say(message)
#         self.tts_engine.runAndWait()

#     def listener_callback(self, msg):
#         try:
#             data = json.loads(msg.data)
#             class_name = data['class_name']
#             distance = data['distance']
#             angle = data['angle']

#             # Check if this object was already spoken about
#             if class_name in self.detected_objects:
#                 return
            
#             # Handle "human" specific condition
#             if class_name == "human" and distance < 2 and -10 <= angle <= 10:
#                 self.speak("Please move aside")
#             else:
#                 self.speak(f"I detected {class_name} located {angle} degrees from me at a distance of {distance} meters.")
#                 self.detected_objects.add(class_name)

#         except Exception as e:
#             self.get_logger().error(f"Failed to process message: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






































import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import edge_tts
from pydub import AudioSegment
from scipy.io import wavfile
import sounddevice as sd
import asyncio

class ObjectSubscriber(Node):
    def __init__(self):
        super().__init__('object_subscriber')
        self.subscription = self.create_subscription(String, '/object_data', self.listener_callback, 10)
        self.detected_objects = set()
        self.get_logger().info("Node initialized and ready to process data")

    async def speak(self, text):
        self.get_logger().info(f'Speaking: {text}')
        """Text-to-Speech function to speak the given text."""
        try:
            tts = edge_tts.Communicate(text, voice="en-GB-LibbyNeural", rate="+1%")
            await tts.save("response.mp3")
            audio_segment = AudioSegment.from_mp3("response.mp3")
            audio_segment.export("response.wav", format="wav")
            samplerate, data = wavfile.read("response.wav")
            sd.play(data, samplerate)
            sd.wait()
        except Exception as e:
            self.get_logger().error(f"Error during playback: {e}")

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            class_name = data['class_name']
            distance = data['distance']
            angle = data['angle']

            # Check if this object was already spoken about
            if class_name in self.detected_objects:
                return

            # Handle "human" specific condition
            if class_name == "human" and distance < 2 and -10 <= angle <= 10:
                asyncio.run(self.speak("Please move aside"))
            else:
                message = f"I detected {class_name} located {angle} degrees from me at a distance of {distance} meters."
                asyncio.run(self.speak(message))
                self.detected_objects.add(class_name)

        except Exception as e:
            self.get_logger().error(f"Failed to process message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
