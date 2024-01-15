import os
import shutil
import torch

class DataSaver:
    def __init__(self, folder_path,file_prefix):
        self.folder_path = folder_path
        self.file_prefix = file_prefix
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

    def _get_next_file_number(self):
        files = [f for f in os.listdir(self.folder_path) if os.path.isfile(os.path.join(self.folder_path, f))]
        max_num = -1
        for file in files:
            if file.startswith(self.file_prefix):
                try:
                    num = int(file[len(self.file_prefix):])
                    max_num = max(max_num, num)
                except ValueError:
                    continue
        return max_num + 1

    def save_file(self, data_collected_input, data_collected_output):
        input_data = torch.tensor(data_collected_input)
        output_data = torch.tensor(data_collected_output)
        next_num = self._get_next_file_number()
        new_file_name = f"{self.file_prefix}{next_num}"
        destination = os.path.join(self.folder_path, new_file_name)
        data_tuple = (input_data, output_data)
        torch.save(data_tuple, destination)

