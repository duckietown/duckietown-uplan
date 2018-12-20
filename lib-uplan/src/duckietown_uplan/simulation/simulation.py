"""
This class is supposed to take care of one simulation / experiment
"""
__all__ = [
    'ConstantProbabiltiySim',
]
from duckietown_uplan.environment.duckie_town import DuckieTown


class ConstantProbabiltiySim(object):
    def __init__(self, current_map, number_of_duckies):
        self.duckie_town = DuckieTown(current_map)
        self.duckie_town.augment_graph()
        self.duckie_town.spawn_random_duckie(number_of_duckies)
        self.duckie_town.get_duckie(0).set_visible_path(True)
        self.duckie_town.reset()
        for i in range(1, len(self.duckie_town.get_duckie_citizens())):
            self.duckie_town.get_duckie(i).stop_movement()

    def execute_simulation(self, time_in_seconds):
        time_per_step = 0.2
        num_of_steps = int(time_in_seconds / time_per_step)
        for i in range(num_of_steps):
            self.duckie_town.create_random_targets_for_all_duckies()
            self.duckie_town.step(time_per_step, display=False)

    def execute_simulation_video(self, time_in_seconds):
        import os
        import cv2
        import shutil
        folder_path = './working_dir'
        if os.path.exists(folder_path):
            shutil.rmtree(folder_path)
        os.mkdir(folder_path)
        video_name = 'simulation_vid.avi'
        time_per_step = 0.2
        num_of_steps = int(time_in_seconds / time_per_step)
        for i in range(num_of_steps):
            self.duckie_town.create_random_targets_for_all_duckies()
            self.duckie_town.step(time_per_step, display=False, save=True, folder='./working_dir', file_index=i)

        images = [img for img in os.listdir(folder_path) if img.endswith(".png")]
        images = sorted(images, key=lambda x: int(os.path.splitext(x)[0].split('file')[1]))
        frame = cv2.imread(os.path.join(folder_path, images[0]))
        height, width, layers = frame.shape
        video = cv2.VideoWriter(video_name, -1, 1, (width, height))
        for image in images:
            video.write(cv2.imread(os.path.join(folder_path, image)))
        cv2.destroyAllWindows()
        video.release()
        shutil.rmtree(folder_path)

    def render_current_state(self):
        self.duckie_town.render_current_graph()

    def reset_simulation(self):
        self.duckie_town.reset()
