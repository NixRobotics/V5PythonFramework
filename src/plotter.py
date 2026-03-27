from vex import *

class XYPlotter:
    def __init__(self):
        # Screen dimensions and margins
        self.screen_width = 480
        self.screen_height = 240
        self.margin_left = 5
        self.margin_right = 5
        self.margin_top = 5
        self.margin_bottom = 5

        # Data series
        self.s1x, self.s1y = [], []
        self.s2x, self.s2y = [], []
        self.s3x, self.s3y = [], []

        # Data limits
        self.min_x = 0
        self.max_x = 100
        self.min_y = 0
        self.max_y = 100

    # -----------------------------
    # Data input
    # -----------------------------
    def add_data_point_series1(self, x, y):
        self.s1x.append(x)
        self.s1y.append(y)

    def add_data_point_series2(self, x, y):
        self.s2x.append(x)
        self.s2y.append(y)

    def add_data_point_series3(self, x, y):
        self.s3x.append(x)
        self.s3y.append(y)

    def clear_data(self):
        self.s1x.clear(); self.s1y.clear()
        self.s2x.clear(); self.s2y.clear()
        self.s3x.clear(); self.s3y.clear()

    # -----------------------------
    # Plotting
    # -----------------------------
    def draw_plot(self, screen: Brain.Lcd):
        """
        screen: an object with methods:
            clear(), draw_rect(x,y,w,h,color),
            draw_circle(x,y,r,color),
            draw_line(x1,y1,x2,y2,color)
        """
        self.update_limits()

        screen.clear_screen()

        plot_x = self.margin_left
        plot_y = self.margin_top
        plot_width = self.screen_width - self.margin_left - self.margin_right
        plot_height = self.screen_height - self.margin_top - self.margin_bottom

        # Draw border
        screen.draw_rectangle(plot_x, plot_y, plot_width, plot_height, Color.BLACK)

        # Draw each series
        self.draw_series(screen, self.s1x, self.s1y, Color.RED)
        self.draw_series(screen, self.s2x, self.s2y, Color.BLUE)
        self.draw_series(screen, self.s3x, self.s3y, Color.GREEN)

    # -----------------------------
    # Scaling and limits
    # -----------------------------
    def update_limits(self):
        xs = self.s1x + self.s2x + self.s3x
        ys = self.s1y + self.s2y + self.s3y

        if not xs or not ys:
            self.min_x, self.max_x = 0, 100
            self.min_y, self.max_y = 0, 100
            return

        self.min_x = min(xs)
        self.max_x = max(xs)
        self.min_y = min(ys)
        self.max_y = max(ys)

        # Avoid zero ranges
        if self.min_x == self.max_x:
            self.max_x += 1
        if self.min_y == self.max_y:
            self.max_y += 1

        print(self.min_x, self.max_x, self.min_y, self.max_y)

    def data_to_screen_x(self, x):
        plot_width = self.screen_width - self.margin_left - self.margin_right
        normalized = (x - self.min_x) / (self.max_x - self.min_x)
        return int(self.margin_left + normalized * plot_width)

    def data_to_screen_y(self, y):
        plot_height = self.screen_height - self.margin_top - self.margin_bottom
        normalized = (y - self.min_y) / (self.max_y - self.min_y)
        return int((self.screen_height - self.margin_bottom) - normalized * plot_height)

    # -----------------------------
    # Draw a series
    # -----------------------------
    def draw_series(self, screen: Brain.Lcd, xs, ys, color):
        if not xs:
            return
        
        screen.set_pen_color(color)

        prev_x = None
        prev_y = None

        for x, y in zip(xs, ys):
            sx = self.data_to_screen_x(x)
            sy = self.data_to_screen_y(y)

            screen.draw_circle(sx, sy, 3, color)

            if prev_x is not None and prev_y is not None:
                screen.draw_line(prev_x, prev_y, sx, sy)

            prev_x, prev_y = sx, sy
