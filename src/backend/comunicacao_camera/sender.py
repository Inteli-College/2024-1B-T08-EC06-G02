import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String  # Corrigido aqui
import cv2
import threading
import time

# Configuração das constantes da câmera
IM_WIDTH = 1280
IM_HEIGHT = 720

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.fps_publisher = self.create_publisher(String, '/fps', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(index=0)
        self.latency_thread = threading.Thread(target=self.latencia)
        self.latency_thread.start()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)

    def latencia(self):
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))

        prev_tick = cv2.getTickCount()
        frame_number, prev_change_frame = 0, 0

        while True:
            frame_number += 1
            ret, frame = self.cap.read()
            if not ret:
                break
            
            new = cv2.getTickCount()

            latency = (new - prev_tick) / cv2.getTickFrequency()
            fps = frame_number - prev_change_frame
            print("{:.3f} sec, {:.3f} frames".format(latency, fps))

            fps_msg = String()
            fps_msg.data = f"{fps} FPS, Latency: {latency:.3f} sec"
            self.fps_publisher.publish(fps_msg)

            prev_tick = new
            prev_change_frame = frame_number

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

