from vex import *

class AllianceColor():
    RED = 0
    BLUE = 1

class AutonSequence():
    SKILLS = 0
    MATCH_LEFT = 1
    MATCH_NONE = 2
    MATCH_RIGHT = 3

class PreAutonUI():
    def __init__(self, brain: Brain, ALLIANCE_COLOR = AllianceColor.RED, AUTON_SEQUENCE = AutonSequence.SKILLS):
        '''
        Docstring for __init__
        
        :param brain: Description
        :param ALLIANCE_COLOR: Description
        :param AUTON_SEQUENCE: Description
        '''
        self.brain = brain # type: Brain
        self.stopping = False
        self.ALLIANCE_COLOR = ALLIANCE_COLOR
        self.AUTON_SEQUENCE = AUTON_SEQUENCE

    def start(self):
        '''
        Docstring for start
        '''
        self.thread = Thread(self._pre_auton_UI)

    def get_current_selection(self) -> tuple:
        '''
        Docstring for get_current_selection
        
        :return: Description
        :rtype: tuple
        '''
        return (self.ALLIANCE_COLOR, self.AUTON_SEQUENCE)
    
    def stop(self):
        '''
        Docstring for stop
        '''
        self.stopping = True
        self.thread.stop()

    def _pre_auton_UI(self):
        # set default mode for UI based on selection at top of file
        mode = 0
        if (self.AUTON_SEQUENCE == AutonSequence.SKILLS):
            mode = 0
        elif (self.AUTON_SEQUENCE == AutonSequence.MATCH_LEFT):
            mode = 1 if (self.ALLIANCE_COLOR == AllianceColor.RED) else 4
        elif (self.AUTON_SEQUENCE == AutonSequence.MATCH_NONE):
            mode = 2 if (self.ALLIANCE_COLOR == AllianceColor.RED) else 5
        elif (self.AUTON_SEQUENCE == AutonSequence.MATCH_RIGHT):
            mode = 3 if (self.ALLIANCE_COLOR == AllianceColor.RED) else 6
        redraw = True
        first_run = True
        # repeat while robot is not enabled
        while not self.stopping:
            # redraw if this is the first run or mode has changed
            if redraw:
                redraw = False
                self.brain.screen.clear_screen()
                if mode == 0:
                    self.brain.screen.set_pen_color(Color.GREEN)
                    self.brain.screen.set_pen_width(10)
                    self.brain.screen.set_fill_color(Color.GREEN)
                    self.brain.screen.draw_rectangle(10, 10, 100, 50)
                    self.ALLIANCE_COLOR = AllianceColor.RED
                    self.AUTON_SEQUENCE = AutonSequence.SKILLS
                elif mode == 1:
                    self.brain.screen.set_pen_color(Color.RED)
                    self.brain.screen.set_pen_width(10)
                    self.brain.screen.set_fill_color(Color.RED)
                    self.brain.screen.draw_rectangle(10, 75, 100, 50)
                    self.ALLIANCE_COLOR = AllianceColor.RED
                    self.AUTON_SEQUENCE = AutonSequence.MATCH_LEFT
                elif mode == 2:
                    self.brain.screen.set_pen_color(Color.RED)
                    self.brain.screen.set_pen_width(10)
                    self.brain.screen.set_fill_color(Color.BLACK)
                    self.brain.screen.draw_rectangle(190, 75, 100, 50)
                    self.ALLIANCE_COLOR = AllianceColor.RED
                    self.AUTON_SEQUENCE = AutonSequence.MATCH_NONE
                elif mode == 3:
                    self.brain.screen.set_pen_color(Color.RED)
                    self.brain.screen.set_pen_width(10)
                    self.brain.screen.set_fill_color(Color.RED)
                    self.brain.screen.draw_rectangle(370, 75, 100, 50)
                    self.ALLIANCE_COLOR = AllianceColor.RED
                    self.AUTON_SEQUENCE = AutonSequence.MATCH_RIGHT
                elif mode == 4:
                    self.brain.screen.set_pen_color(Color.BLUE)
                    self.brain.screen.set_pen_width(10)
                    self.brain.screen.set_fill_color(Color.BLUE)
                    self.brain.screen.draw_rectangle(10, 150, 100, 50)
                    self.ALLIANCE_COLOR = AllianceColor.BLUE
                    self.AUTON_SEQUENCE = AutonSequence.MATCH_LEFT
                elif mode == 5:
                    self.brain.screen.set_pen_color(Color.BLUE)
                    self.brain.screen.set_pen_width(10)
                    self.brain.screen.set_fill_color(Color.BLACK)
                    self.brain.screen.draw_rectangle(190, 150, 100, 50)
                    self.ALLIANCE_COLOR = AllianceColor.BLUE
                    self.AUTON_SEQUENCE = AutonSequence.MATCH_NONE
                elif mode == 6:
                    self.brain.screen.set_pen_color(Color.BLUE)
                    self.brain.screen.set_pen_width(10)
                    self.brain.screen.set_fill_color(Color.BLUE)
                    self.brain.screen.draw_rectangle(370, 150, 100, 50)
                    self.ALLIANCE_COLOR = AllianceColor.BLUE
                    self.AUTON_SEQUENCE = AutonSequence.MATCH_RIGHT

                # If we are redrawing (and not furst run), then add a debounce    
                if not first_run:
                    wait(0.5, SECONDS) # debounce
                first_run = False

            wait(0.01, SECONDS)

            # Check if screen was pressed somewhere
            if self.brain.screen.pressing():
                mode += 1
                redraw = True
                if mode > 6:
                    mode = 0
