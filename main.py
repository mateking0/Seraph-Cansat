from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.button import Button
from kivy.uix.filechooser import FileChooserListView
from kivy.uix.progressbar import ProgressBar
from kivy.clock import Clock
from kivy.clock import mainthread
import cv2
import numpy as np
import _thread


class RGBProcessingApp(App):

    def build(self):
        layout = GridLayout(cols=2, spacing=10, padding=20)

        button_explore = Button(text="Browse Files")
        button_explore.bind(on_press=self.browse_files)
        layout.add_widget(button_explore)

        self.selected_file_label = Label()
        layout.add_widget(self.selected_file_label)

        run_button = Button(text='Run', on_press=self.start_image_processing)
        run_button.bind()
        layout.add_widget(run_button)

        self.progress_bar = ProgressBar()
        layout.add_widget(self.progress_bar)

        self.lower_bound = None
        self.upper_bound = None

        rgb_data = np.array([
            [68, 166, 115],
            [111, 145, 59],
            [58, 100, 39],
            [29, 91, 79],
            [24, 142, 116],
            [59, 154, 54],
            [17, 165, 140],
            [34, 117, 42],
            [23, 135, 118],
            [33, 170, 49],
            [66, 180, 75],
            [108, 173, 81],
            [105, 135, 51],
            [42, 186, 155],
            [63, 159, 81],
            [37, 97, 39],
            [39, 96, 54],
            [40, 126, 104],
            [86, 150, 32],
            [98, 177, 37],
            [115, 149, 58],
            [78, 121, 45],
            [28, 91, 47],
            [38, 108, 85],
            [12, 90, 74],
            [72, 110, 26],
            [7, 173, 114],
            [27, 93, 61],
            [43, 191, 95],
            [41, 170, 139],
            [57, 166, 116],
            [23, 141, 97],
            [62, 173, 126],
            [48, 182, 148],
            [59, 96, 20],
            [5, 109, 86],
            [4, 86, 60],
            [67, 81, 26],
            [128, 151, 36],
            [37, 84, 45],
            [37, 82, 28],
            [25, 128, 82],
            [77, 123, 1],
            [122, 168, 61],
            [34, 127, 109],
            [41, 117, 102],
            [91, 194, 116],
            [28, 154, 89],
            [142, 168, 56],
            [17, 150, 121],
            [77, 194, 134],
            [73, 198, 60],
            [26, 123, 81],
            [56, 115, 56],
            [46, 116, 87],
            [36, 91, 56],
            [45, 96, 58],
            [69, 112, 49],
            [127, 177, 64],
            [52, 97, 47],
            [72, 147, 72],
            [103, 140, 40],
            [66, 133, 81],
            [84, 146, 64],
            [18, 105, 87],
            [23, 97, 84],
            [62, 122, 73],
            [74, 158, 109],
            [40, 123, 105],
            [36, 176, 148],
            [66, 88, 0],
            [119, 146, 24],
            [50, 141, 124],
            [72, 103, 37],
            [6, 120, 90],
            [21, 91, 76],
            [37, 109, 95],
            [47, 138, 77],
            [58, 165, 113],
            [69, 112, 48],
            [84, 200, 95],
            [87, 192, 155],
            [1, 120, 64],
            [84, 181, 96],
            [66, 191, 76],
            [149, 183, 25],
            [52, 163, 121],
            [60, 102, 35],
            [10, 126, 101],
            [102, 179, 97],
            [60, 114, 18],
            [51, 158, 134],
            [35, 123, 101],
            [53, 193, 79],
            [0, 117, 99],
            [31, 137, 102],
            [30, 91, 64],
            [39, 122, 69],
            [47, 130, 65],
            [175, 255, 241],
            [206, 255, 178],
            [170, 255, 198],
            [189, 239, 107],
            [22, 25, 20],
            [68, 79, 70],
            [127, 163, 129],
            [206, 234, 187],
            [2, 5, 4],
            [17, 30, 27],
            [22, 35, 32],
            [164, 249, 206],
            [198, 255, 158],
            [156, 252, 191],
            [174, 252, 201],
            [99, 117, 91],
            [23, 38, 13],
            [10, 73, 49],
            [86, 137, 103],
            [117, 150, 117],
            [88, 140, 81],
            [67, 73, 59],
            [216, 255, 173],
            [221, 255, 239],
            [104, 124, 111],
            [56, 124, 63],
            [154, 214, 170],
            [108, 135, 82],
            [5, 56, 33],
            [26, 33, 30],
            [37, 86, 61],
            [35, 119, 52],
            [12, 22, 13],
            [136, 163, 127],
            [32, 56, 15],
            [12, 35, 34],
            [100, 153, 101],
            [114, 145, 104],
            [0, 10, 1],
            [91, 119, 61],
            [16, 28, 23],
            [160, 229, 135],
            [1, 30, 5],
            [13, 38, 0],
            [18, 56, 2],
            [6, 20, 12],
            [41, 63, 29],
            [35, 66, 26],
            [16, 25, 18],
            [10, 33, 13],
            [45, 73, 74],
            [2, 58, 24],
            [11, 28, 2],
            [190, 226, 202],
            [15, 30, 19],
            [59, 76, 29],
            [171, 232, 172],
            [113, 140, 119],
            [5, 28, 8],
            [12, 71, 15],
            [137, 242, 46],
            [101, 140, 47],
            [55, 137, 30],
            [118, 188, 142],
            [76, 150, 102],
            [117, 196, 121],
            [78, 145, 97],
            [14, 35, 22],
            [81, 107, 61],
            [29, 130, 98],
            [153, 178, 158],
            [187, 249, 62],
            [159, 239, 79],
            [88, 160, 53],
            [129, 226, 143],
            [56, 112, 97],
            [121, 178, 153],
            [92, 117, 80],
            [11, 35, 29],
            [20, 104, 48],
            [94, 112, 94],
            [50, 130, 82],
            [78, 107, 32],
            [197, 239, 191],
            [43, 135, 29],
            [83, 119, 45],
            [17, 61, 56],
            [85, 112, 67],
            [23, 107, 0],
            [42, 53, 2],
            [77, 119, 56],
            [127, 168, 128],
            [85, 117, 58],
            [146, 191, 80],
            [162, 214, 175],
            [67, 109, 55],
            [25, 13, 59],
            [153, 224, 53],
            [75, 145, 100],
            [168, 224, 154],
            [10, 38, 34],
            [173, 131, 158],
            [61, 24, 33],
            [182, 157, 204],
            [66, 3, 132],
            [7, 37, 63],
            [148, 144, 221],
            [99, 139, 147],
            [229, 45, 110],
            [175, 140, 144],
            [108, 98, 109],
            [122, 106, 116],
            [1, 1, 2],
            [249, 125, 17],
            [167, 176, 211],
            [158, 184, 211],
            [186, 161, 107],
            [219, 128, 17],
            [151, 97, 168],
            [119, 15, 9],
            [117, 53, 70],
            [96, 18, 96],
            [132, 108, 103],
            [193, 177, 116],
            [77, 163, 188],
            [255, 252, 235],
            [53, 40, 25],
            [110, 126, 160],
            [168, 120, 115],
            [0, 0, 0],
            [182, 132, 52],
            [117, 28, 38],
            [207, 91, 97],
            [72, 49, 73],
            [9, 60, 80],
            [111, 116, 117],
            [206, 227, 254],
            [224, 227, 255],
            [2, 16, 20],
            [58, 60, 102],
            [26, 62, 80],
            [150, 116, 74],
            [200, 5, 171],
            [212, 217, 220],
            [42, 57, 83],
            [18, 69, 121],
            [221, 206, 209],
            [159, 139, 143],
            [102, 54, 137],
            [206, 205, 127],
            [207, 134, 115],
            [189, 13, 117],
            [253, 252, 252],
            [20, 24, 73],
            [91, 223, 215],
            [120, 175, 230],
            [192, 222, 247],
            [91, 223, 215],
            [120, 175, 230],
            [192, 222, 247],
            [247, 246, 242],
            [183, 166, 219],
            [114, 103, 71],
            [163, 122, 18],
            [142, 120, 159],
            [149, 101, 149],
            [223, 187, 177],
            [245, 231, 252],
            [225, 239, 248],
            [3, 40, 53],
            [210, 202, 208],
            [176, 133, 99],
            [214, 90, 205],
            [200, 141, 202],
            [194, 195, 202],
            [1, 1, 1],
            [86, 120, 124],
            [136, 107, 121],
            [234, 205, 196],
            [152, 163, 165],
            [237, 199, 188],
            [66, 57, 140],
            [236, 220, 229],
            [149, 137, 104],
            [160, 146, 254],
            [147, 153, 249],
            [65, 52, 36],
            [111, 67, 112],
            [228, 218, 230],
            [1, 25, 71],
            [82, 25, 169],
            [2, 2, 2],
            [30, 95, 190],
            [229, 184, 243],
            [217, 204, 217],
            [252, 130, 145],
            [42, 7, 16],
            [229, 93, 47],
            [150, 125, 129],
            [156, 74, 22],
            [148, 169, 202],
            [131, 101, 64],
            [191, 151, 192],
            [222, 255, 248],
            [67, 65, 65],
            [253, 165, 124],
            [69, 244, 208],
            [164, 113, 20],
            [34, 96, 151],
            [66, 52, 66],
            [107, 50, 85],
            [255, 157, 101],
            [20, 14, 29],
            [215, 217, 248],
            [160, 124, 175],
            [56, 48, 50],
            [158, 177, 187],
            [238, 236, 220],
            [93, 47, 92],
            [229, 170, 107],
            [113, 58, 80],
            [161, 194, 233],
            [3, 25, 81],
            [240, 251, 250],
            [105, 9, 98],
            [124, 93, 195],
            [176, 68, 60],
            [49, 36, 200],
            [47, 9, 55],
            [250, 246, 253],
            [189, 103, 53],
            [23, 21, 24],
            [10, 202, 214],
            [100, 77, 45],
            [154, 13, 99],
            [37, 35, 36],
            [82, 145, 168],
            [112, 59, 81],
            [243, 226, 206],
            [199, 196, 182],
            [42, 34, 74],
            [19, 13, 32],
            [136, 71, 136],
            [118, 113, 99],
            [244, 233, 229],
            [227, 172, 165],
            [65, 138, 142],
            [222, 236, 249],
            [238, 157, 81],
            [244, 203, 200],
            [225, 178, 171],
            [186, 160, 195],
            [95, 88, 66],
            [46, 212, 214],
            [172, 165, 38],
            [83, 86, 109],
            [144, 98, 98],
            [247, 245, 243],
            [28, 15, 20],
            [86, 65, 83],
            [42, 92, 109],
            [198, 112, 135],
            [0, 19, 33],
            [107, 62, 8],
            [247, 243, 234],
            [200, 115, 248],
            [195, 201, 130],
            [173, 173, 119],
            [140, 123, 74],
            [97, 28, 216],
            [33, 32, 35],
            [162, 174, 214],
            [227, 239, 236],
            [8, 4, 66],
            [144, 165, 165],
            [237, 208, 226],
            [1, 1, 4],
            [232, 224, 231],
            [46, 19, 21],
            [4, 36, 42],
            [146, 104, 5],
            [148, 214, 233],
            [194, 85, 26],
            [128, 41, 92],
            [186, 161, 128],
            [218, 226, 219],
            [185, 198, 5],
            [118, 110, 171],
            [247, 233, 251],
            [149, 231, 245],
            [225, 254, 155],
            [99, 101, 103],
            [244, 75, 212],
            [165, 154, 171],
            [188, 147, 118],
            [149, 61, 165],
            [188, 147, 118],
            [219, 219, 219],
            [39, 22, 75],
            [160, 121, 6],
            [222, 191, 219],
            [191, 181, 105],
            [89, 79, 45],
            [224, 216, 190],
            [206, 205, 198],
            [158, 157, 139],
            [219, 194, 103],
            [234, 219, 11],
            [186, 188, 37],
            [122, 111, 44],
            [25, 2, 229],
            [34, 145, 214],
            [5, 9, 80],
            [138, 181, 242],
            [129, 194, 196],
            [94, 77, 165],
            [6, 122, 122],
            [22, 138, 170],
            [114, 106, 204],
            [131, 127, 142],
            [110, 94, 255],
            [12, 216, 252],
            [57, 144, 150],
            [74, 70, 201],
            [84, 102, 153],
            [146, 170, 229],
            [76, 78, 147],
            [46, 216, 219],
            [201, 172, 78],
            [81, 81, 37],
            [219, 187, 28],
            [227, 229, 174],
            [142, 130, 19],
            [102, 101, 92],
            [155, 148, 18],
            [175, 175, 142],
            [214, 173, 8],
            [206, 203, 183],
            [154, 155, 97],
            [146, 147, 105],
            [191, 182, 133],
            [229, 209, 144],
            [193, 193, 147],
            [1450, 150, 3],
            [119, 112, 87],
            [130, 129, 84],
            [124, 116, 52],
            [237, 230, 45],
            [147, 132, 66],
            [216, 199, 43],
            [77, 140, 142],
            [81, 101, 119],
            [26, 91, 221],
            [25, 30, 122],
            [186, 219, 242],
            [227, 227, 237],
            [135, 133, 147],
            [157, 154, 209],
            [137, 112, 229],
            [164, 184, 242],
            [100, 123, 234],
            [175, 187, 198],
            [166, 159, 242],
            [179, 221, 252],
            [183, 175, 209],
            [132, 132, 132],
            [127, 127, 127],
            [175, 175, 175],
            [173, 173, 173],
            [104, 104, 104],
            [201, 201, 201],
            [186, 186, 186],
            [226, 226, 226],
            [232, 232, 232],
            [121, 113, 122],
            [169, 40, 183],
            [130, 84, 125],
            [193, 145, 167],
            [236, 203, 242],
            [147, 142, 146],
            [163, 94, 178],
            [183, 91, 175],
            [106, 73, 114],
            [110, 22, 147],
            [114, 20, 108],
            [175, 143, 124],
            [163, 144, 119],
            [132, 97, 37],
            [188, 186, 183],
            [209, 178, 117],
            [196, 187, 164],
            [132, 110, 53],
            [201, 141, 80],
            [150, 145, 141],
            [170, 122, 59],
            [244, 206, 144],
            [186, 68, 5],
            [211, 145, 23],
            [204, 185, 126],
            [107, 83, 71],
            [196, 141, 113],
            [221, 170, 133],
            [132, 103, 30],
            [155, 104, 27],
            [104, 61, 30],
            [221, 138, 93],
            [239, 57, 96],
            [193, 178, 181],
            [175, 172, 173],
            [211, 92, 71],
            [168, 127, 122],
            [127, 7, 23],
            [252, 166, 202],
            [216, 13, 16],
            [135, 105, 110],
            [188, 104, 79],
            [204, 164, 153],
            [196, 178, 179],
            [153, 13, 55],
            [127, 95, 104],
            [219, 87, 142],
            [163, 86, 108],
            [11, 185, 191],
            [118, 118, 122],
            [98, 96, 109],
            [133, 142, 188],
            [2, 4, 135],
            [33, 116, 137],
            [26, 102, 242],
            [94, 100, 130],
            [158, 145, 193],
            [68, 73, 137],
            [133, 142, 155],
            [188, 180, 145],
            [224, 221, 170],
            [255, 230, 107],
            [252, 247, 146],
            [127, 126, 108],
            [198, 191, 59],
            [239, 237, 203],
            [145, 133, 87],
            [32, 88, 70],
            [22, 83, 68],
            [13, 86, 70],
            [41, 91, 50],
            [49, 94, 51],
            [53, 97, 47],
            [58, 104, 51],
            [55, 102, 49],
            [28, 51, 5],
            [21, 68, 17],
            [3, 17, 4],
            [5, 22, 10],
            [0, 33, 29],
            [0, 33, 4],
            [21, 35, 19],
            [19, 63, 50],
            [25, 53, 36],
            [25, 45, 19],
            [12, 33, 17],
            [9, 51, 34],
            [68, 94, 13],
            [16, 58, 7],
            [30, 150, 42],
            [154, 214, 44],
            [51, 127, 79],
            [11, 43, 34],
            [6, 81, 52],
            [199, 249, 144],
            [155, 191, 120],
            [7, 104, 15],
            [94, 130, 101],
            [19, 35, 27],
            [112, 140, 100],
            [13, 45, 21],
            [63, 79, 65],
            [7, 28, 15],
            [23, 53, 29],
            [17, 40, 15],
            [148, 211, 80],
            [1, 40, 13],
            [51, 101, 74],
            [77, 75, 62],
            [62, 72, 38],
            [66, 102, 51],
            [0, 109, 87],
            [49, 67, 59],
            [32, 66, 48],
            [77, 89, 67],
            [40, 75, 62],
            [0, 51, 0],
            [31, 46, 4],
            [25, 50, 50],
            [12, 36, 11],
            [38, 60, 41],
            [40, 108, 86],
            [143, 188, 143],
            [17, 93, 51],
            [58, 65, 50],
            [154, 205, 50],
            [54, 66, 47],
            [79, 121, 66],
            [81, 83, 78],
            [27, 121, 49],
            [0, 168, 107],
            [16, 43, 13],
            [85, 119, 73],
            [0, 38, 14],
            [8, 40, 10],
            [19, 35, 22],
            [35, 61, 31],
            [25, 51, 18],
            [31, 88, 68],
            [19, 107, 68],
            [29, 89, 60],
            [12, 83, 58],
            [70, 114, 47],
            [0, 153, 40],
            [9, 130, 89],
            [13, 137, 81],
            [1, 127, 39],
            [1, 104, 3],
            [5, 117, 44],
            [3, 30, 26],
            [43, 63, 40],
            [43, 84, 51],
            [5, 81, 51],
            [49, 58, 33],
            [93, 112, 77],
            [161, 193, 151],
            [16, 25, 17],
            [15, 35, 31],
            [57, 81, 61],
            [62, 99, 62],
            [64, 113, 20],
            [9, 30, 18],
            [17, 48, 16],
            [9, 48, 36],
            [7, 22, 8],
            [3, 22, 17],
            [53, 104, 75],
            [163, 186, 170],
            [23, 35, 23],
            [17, 68, 19],
            [26, 35, 27],
            [69, 94, 79],
            [157, 186, 122],
            [21, 51, 1],
            [146, 201, 141],
            [5, 28, 8],
            [110, 178, 116],
            [123, 160, 22],
            [187, 255, 158],
            [4, 244, 92],
            [26, 130, 43],
            [128, 165, 127],
            [78, 165, 6],
            [1, 38, 5],
            [111, 229, 43],
            [1, 33, 18],
            [109, 168, 116],
            [33, 122, 65],
            [64, 132, 55],
            [22, 122, 48],
            [36, 51, 30],
            [39, 63, 2],
            [3, 96, 64],
            [12, 35, 4],
            [144, 165, 137],
            [48, 155, 98],
            [45, 84, 49],
            [11, 193, 127],
            [83, 158, 26],
            [75, 168, 95],
            [38, 68, 35],
            [81, 124, 89],
            [194, 252, 143],
            [5, 76, 52],
            [93, 140, 39],
            [74, 109, 64],
            [178, 219, 173]

        ])
        labels = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           1, 1, 1, 1, 1, 1, 1, 1, 1, 1
                           ])

        x_train, x_test, y_train, y_test = train_test_split(rgb_data, labels, test_size=0.2, random_state=42)
        self.model = RandomForestClassifier(n_estimators=100, random_state=42)
        self.model.fit(x_train, y_train)

        return layout

    def start_image_processing(self, instance, *args):
        _thread.start_new_thread(self.process_image, ())

    def process_image(self, *args):
        image = cv2.imread(self.pathtofile)
        height, width, _ = image.shape
        # hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        passed_pixels = 0
        pixels = 0
        image_size = height * width

        def update_progress_bar(dt):
            self.progress_bar.value = (pixels / image_size) * 100

        Clock.schedule_interval(update_progress_bar, 0.01)
        kiir = open("output.txt",'w')
        for y in range(height):
            for x in range(width):
                pixel_value_bgr = image[y, x]
                pixel_value_rgb = cv2.cvtColor(np.array([[pixel_value_bgr]]), cv2.COLOR_BGR2RGB)[0][0]
                if self.isgreen(np.array([[pixel_value_rgb[0], pixel_value_rgb[1], pixel_value_rgb[2]]])):
                    passed_pixels += 1
                    print("#",file=kiir,end="")
                else:
                    print(" ",file=kiir,end="")
                pixels += 1
                self.progress_bar.value = (pixels / image_size) * 100
            print(file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        print("",file=kiir)
        kiir.close()
        self.progress_bar.value = 100
        Clock.schedule_once(self.show_popup_in_main_thread(message=f'{round((passed_pixels / image_size) * 100, 4)}%'))

    @mainthread
    def show_popup_in_main_thread(self, *args, message):
        self.show_popup("Result:", message)

    def isgreen(self, rgb_input):
        prediction = self.model.predict(rgb_input)
        if prediction == 1:
            return True
        else:
            return False

    def show_popup(self, title, message):
        popup = Popup(title=title, content=Label(text=message), size_hint=(None, None), size=(400, 200))
        popup.open()

    def browse_files(self, instance):
        file_chooser = FileChooserListView()
        file_chooser.bind(on_submit=self.on_file_selected)
        self.popup = Popup(title="Select a file", content=file_chooser, size_hint=(0.9, 0.9))
        self.popup.open()

    def on_file_selected(self, instance, selection, *args):
        self.popup.dismiss()

        if len(selection) > 0:
            self.pathtofile = selection[0]
            self.selected_file_label.text = selection[0]
        else:
            self.show_popup('Hi', 'secret message from developer: Idk how you made this happen')


if __name__ == '__main__':
    RGBProcessingApp().run()

