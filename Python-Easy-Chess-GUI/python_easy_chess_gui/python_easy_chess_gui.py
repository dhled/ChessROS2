#!/usr/bin/env python3
""" 
python_easy_chess_gui.py

Requirements:
    Python 3.7.3 and up

PySimpleGUI Square Mapping
board = [
    56, 57, ... 63
    ...
    8, 9, ...
    0, 1, 2, ...
]

row = [
    0, 0, ...
    1, 1, ...
    ...
    7, 7 ...
]

col = [
    0, 1, 2, ... 7
    0, 1, 2, ...
    ...
    0, 1, 2, ... 7
]


Python-Chess Square Mapping
board is the same as in PySimpleGUI
row is reversed
col is the same as in PySimpleGUI

"""

import PySimpleGUI as sg
import os
import sys
import subprocess
import threading
from pathlib import Path, PurePath  # Python 3.4 and up
import queue
import copy
import time
from datetime import datetime
import json
import pyperclip
import chess
import chess.pgn
import chess.engine
import chess.polyglot
import logging
import platform as sys_plat

import rclpy
from rclpy.node import Node
from chess_msgs.srv._get_next_move import GetNextMove_Request
from chess_msgs.srv import GetNextMove, SetEloRating
from chess_msgs.msg import ChessMove
from std_srvs.srv import Empty as EmptyRq
from std_msgs.msg import Empty
from ament_index_python.packages import get_package_share_directory
import threading


log_format = "%(asctime)s :: %(funcName)s :: line: %(lineno)d :: %(" "levelname)s :: %(message)s"
logging.basicConfig(filename="pecg_log.txt", filemode="w", level=logging.DEBUG, format=log_format)


APP_NAME = "Python Easy Chess GUI"
APP_VERSION = "v1.14"
BOX_TITLE = "{} {}".format(APP_NAME, APP_VERSION)


platform = sys.platform
sys_os = sys_plat.system()


ico_path = {
    "win32": {"pecg": "Icon/pecg.ico", "enemy": "Icon/enemy.ico", "adviser": "Icon/adviser.ico"},
    "linux": {"pecg": "Icon/pecg.png", "enemy": "Icon/enemy.png", "adviser": "Icon/adviser.png"},
    "darwin": {"pecg": "Icon/pecg.png", "enemy": "Icon/enemy.png", "adviser": "Icon/adviser.png"},
}


MIN_DEPTH = 1
MAX_DEPTH = 1000
MANAGED_UCI_OPTIONS = ["ponder", "uci_chess960", "multipv", "uci_analysemode", "ownbook"]
GUI_THEME = [
    "Green",
    "GreenTan",
    "LightGreen",
    "BluePurple",
    "Purple",
    "BlueMono",
    "GreenMono",
    "BrownBlue",
    "BrightColors",
    "NeutralBlue",
    "Kayak",
    "SandyBeach",
    "TealMono",
    "Topanga",
    "Dark",
    "Black",
    "DarkAmber",
]
IMAGE_PATH = os.path.join(get_package_share_directory("python_easy_chess_gui"), "Images")  # path to the chess pieces


BLANK = 0  # piece names
PAWNB = 1
KNIGHTB = 2
BISHOPB = 3
ROOKB = 4
KINGB = 5
QUEENB = 6
PAWNW = 7
KNIGHTW = 8
BISHOPW = 9
ROOKW = 10
KINGW = 11
QUEENW = 12


# Absolute rank based on real chess board, white at bottom, black at the top.
# This is also the rank mapping used by python-chess modules.
RANK_8 = 7
RANK_7 = 6
RANK_6 = 5
RANK_5 = 4
RANK_4 = 3
RANK_3 = 2
RANK_2 = 1
RANK_1 = 0


initial_board = [
    [ROOKB, KNIGHTB, BISHOPB, QUEENB, KINGB, BISHOPB, KNIGHTB, ROOKB],
    [
        PAWNB,
    ]
    * 8,
    [
        BLANK,
    ]
    * 8,
    [
        BLANK,
    ]
    * 8,
    [
        BLANK,
    ]
    * 8,
    [
        BLANK,
    ]
    * 8,
    [
        PAWNW,
    ]
    * 8,
    [ROOKW, KNIGHTW, BISHOPW, QUEENW, KINGW, BISHOPW, KNIGHTW, ROOKW],
]


white_init_promote_board = [[QUEENW, ROOKW, BISHOPW, KNIGHTW]]

black_init_promote_board = [[QUEENB, ROOKB, BISHOPB, KNIGHTB]]


HELP_MSG = """(A) To play a game
You should be in Play mode.
1. Mode->Play
2. Make move on the board

(B) To play as black
You should be in Neutral mode
1. Board->Flip
2. Mode->Play
3. Engine->Go
If you are already in Play mode, go back to 
Neutral mode via Mode->Neutral

(C) To flip board
You should be in Neutral mode
1. Board->Flip
  
(D) To paste FEN
You should be in Play mode
1. Mode->Play
2. FEN->Paste

(E) To show engine search info after the move                
1. Right-click on the Opponent Search Info and press Show

(F) To Show book 1 and 2
1. Right-click on Book 1 or 2 press Show
"""


# Images/60
blank = os.path.join(IMAGE_PATH, "blank.png")
bishopB = os.path.join(IMAGE_PATH, "bB.png")
bishopW = os.path.join(IMAGE_PATH, "wB.png")
pawnB = os.path.join(IMAGE_PATH, "bP.png")
pawnW = os.path.join(IMAGE_PATH, "wP.png")
knightB = os.path.join(IMAGE_PATH, "bN.png")
knightW = os.path.join(IMAGE_PATH, "wN.png")
rookB = os.path.join(IMAGE_PATH, "bR.png")
rookW = os.path.join(IMAGE_PATH, "wR.png")
queenB = os.path.join(IMAGE_PATH, "bQ.png")
queenW = os.path.join(IMAGE_PATH, "wQ.png")
kingB = os.path.join(IMAGE_PATH, "bK.png")
kingW = os.path.join(IMAGE_PATH, "wK.png")


images = {
    BISHOPB: bishopB,
    BISHOPW: bishopW,
    PAWNB: pawnB,
    PAWNW: pawnW,
    KNIGHTB: knightB,
    KNIGHTW: knightW,
    ROOKB: rookB,
    ROOKW: rookW,
    KINGB: kingB,
    KINGW: kingW,
    QUEENB: queenB,
    QUEENW: queenW,
    BLANK: blank,
}


# Promote piece from psg (pysimplegui) to pyc (python-chess)
promote_psg_to_pyc = {
    KNIGHTB: chess.KNIGHT,
    BISHOPB: chess.BISHOP,
    ROOKB: chess.ROOK,
    QUEENB: chess.QUEEN,
    KNIGHTW: chess.KNIGHT,
    BISHOPW: chess.BISHOP,
    ROOKW: chess.ROOK,
    QUEENW: chess.QUEEN,
}


INIT_PGN_TAG = {
    "Event": "Human vs computer",
    "White": "Human",
    "Black": "Computer",
}


# (1) Mode: Neutral
menu_def_neutral = [
    ["&Mode", ["Play"]],
    ["Boar&d", ["Flip", "Color", ["Brown::board_color_k", "Blue::board_color_k", "Green::board_color_k", "Gray::board_color_k"], "Theme", GUI_THEME]],
    ["&Settings", ["Game::settings_game_k"]],
    ["&Help", ["About"]],
]

# (2) Mode: Play, info: hide
menu_def_play = [
    ["&Mode", ["Neutral"]],
    [
        "&Game",
        [
            "&New::new_game_k",
            "Resign::resign_game_k",
            "User Wins::user_wins_k",
            "User Draws::user_draws_k",
        ],
    ],
    ["&Help", ["About"]],
]


class EasyChessGui(Node):
    queue = queue.Queue()
    is_user_white = True  # White is at the bottom in board layout

    def __init__(
        self,
        theme,
    ):
        super().__init__("chess_gui")
        self.theme = theme
        self.opp_path_and_file = None
        self.opp_file = None
        self.opp_id_name = None
        self.adviser_file = None
        self.adviser_path_and_file = None
        self.adviser_id_name = None
        self.adviser_hash = 128
        self.adviser_threads = 1
        self.adviser_movetime_sec = 10
        self.pecg_auto_save_game = "pecg_auto_save_games.pgn"
        self.my_games = "pecg_my_games.pgn"
        self.init_game()
        self.fen = None
        self.psg_board = None
        self.menu_elem = None
        self._robot_moving = False
        self.engine_id_name_list = []
        self.engine_file_list = []
        self._robot_done = self.create_subscription(Empty, "robot_moved", self._robot_done_cb, 1)
        self._played_move_publisher = self.create_publisher(ChessMove, "played_move", 1)
        self._get_next_move = self.create_client(GetNextMove, "get_next_move")
        self._reset_game = self.create_client(EmptyRq, "reset_game")

        self.human_base_time_ms = 5 * 60 * 1000  # 5 minutes
        self.human_inc_time_ms = 10 * 1000  # 10 seconds
        self.human_period_moves = 0
        self.human_tc_type = "fischer"

        self.engine_base_time_ms = 3 * 60 * 1000  # 5 minutes
        self.engine_inc_time_ms = 2 * 1000  # 10 seconds
        self.engine_period_moves = 0
        self.engine_tc_type = "fischer"

        # Default board color is brown
        self.sq_light_color = "#F0D9B5"
        self.sq_dark_color = "#B58863"

        # Move highlight, for brown board
        self.move_sq_light_color = "#E8E18E"
        self.move_sq_dark_color = "#B8AF4E"

        self.gui_theme = "Reddit"

        self.is_save_time_left = False
        self.is_save_user_comment = True

    def _robot_done_cb(self, _):
        if self._robot_moving:
            self._robot_moving = False

    def update_game(self, user_move, type):
        """
        Used for saving moves in the game.

        :param mc: move count
        :param user_move:
        :param time_left:
        :param user_comment: Can be a 'book' from the engine
        :return:
        """
        self.node = self.game.add_variation(user_move)
        move = ChessMove()
        move.move = str(user_move)
        move.type = type

        self._played_move_publisher.publish(move)
        self._robot_moving = True

    def create_new_window(self, window, flip=False):
        """ Close the window param just before turning the new window """

        loc = window.CurrentLocation()
        window.Disable()
        if flip:
            self.is_user_white = not self.is_user_white

        layout = self.build_main_layout(self.is_user_white)

        w = sg.Window(
            "{} {}".format(APP_NAME, APP_VERSION),
            layout,
            default_button_element_size=(12, 1),
            auto_size_buttons=False,
            location=(loc[0], loc[1]),
            icon=ico_path[platform]["pecg"],
        )

        # Initialize White and black boxes
        while True:
            button, value = w.Read(timeout=50)
            break

        window.Close()

    def get_time_mm_ss_ms(self, time_ms):
        """ Returns time in min:sec:millisec given time in millisec """
        s, ms = divmod(int(time_ms), 1000)
        m, s = divmod(s, 60)

        # return '{:02d}m:{:02d}s:{:03d}ms'.format(m, s, ms)
        return "{:02d}m:{:02d}s".format(m, s)

    def get_time_h_mm_ss(self, time_ms, symbol=True):
        """
        Returns time in h:mm:ss format.

        :param time_ms:
        :param symbol:
        :return:
        """
        s, ms = divmod(int(time_ms), 1000)
        m, s = divmod(s, 60)
        h, m = divmod(m, 60)

        if not symbol:
            return "{:01d}:{:02d}:{:02d}".format(h, m, s)
        return "{:01d}h:{:02d}m:{:02d}s".format(h, m, s)

    def get_tag_date(self):
        """ Return date in pgn tag date format """
        return datetime.today().strftime("%Y.%m.%d")

    def init_game(self):
        """ Initialize game with initial pgn tag values """
        self.game = chess.pgn.Game()
        self.node = None
        self.game.headers["Event"] = INIT_PGN_TAG["Event"]
        self.game.headers["Date"] = self.get_tag_date()
        self.game.headers["White"] = INIT_PGN_TAG["White"]
        self.game.headers["Black"] = INIT_PGN_TAG["Black"]

    def set_new_game(self):
        """ Initialize new game but save old pgn tag values"""
        old_event = self.game.headers["Event"]
        old_white = self.game.headers["White"]
        old_black = self.game.headers["Black"]

        # Define a game object for saving game in pgn format
        self.game = chess.pgn.Game()

        self.game.headers["Event"] = old_event
        self.game.headers["Date"] = self.get_tag_date()
        self.game.headers["White"] = old_white
        self.game.headers["Black"] = old_black

    def clear_elements(self, window):
        """ Clear movelist, score, pv, time, depth and nps boxes """
        window.find_element("search_info_all_k").Update("")
        window.find_element("_movelist_").Update(disabled=False)
        window.find_element("_movelist_").Update("", disabled=True)
        window.find_element("polyglot_book1_k").Update("")
        window.find_element("polyglot_book2_k").Update("")
        window.find_element("advise_info_k").Update("")
        window.find_element("comment_k").Update("")
        window.Element("w_base_time_k").Update("")
        window.Element("b_base_time_k").Update("")
        window.Element("w_elapse_k").Update("")
        window.Element("b_elapse_k").Update("")

    def update_labels_and_game_tags(self, window, human="Human"):
        """ Update player names """
        engine_id = self.opp_id_name
        if self.is_user_white:
            window.find_element("_White_").Update(human)
            window.find_element("_Black_").Update(engine_id)
            self.game.headers["White"] = human
            self.game.headers["Black"] = engine_id
        else:
            window.find_element("_White_").Update(engine_id)
            window.find_element("_Black_").Update(human)
            self.game.headers["White"] = engine_id
            self.game.headers["Black"] = human

    def change_square_color(self, window, row, col):
        """
        Change the color of a square based on square row and col.
        """
        btn_sq = window.find_element(key=(row, col))
        is_dark_square = True if (row + col) % 2 else False
        bd_sq_color = self.move_sq_dark_color if is_dark_square else self.move_sq_light_color
        btn_sq.Update(button_color=("white", bd_sq_color))

    def relative_row(self, s, stm):
        """
        The board can be viewed, as white at the bottom and black at the
        top. If stm is white the row 0 is at the bottom. If stm is black
        row 0 is at the top.
        :param s: square
        :param stm: side to move
        :return: relative row
        """
        return 7 - self.get_row(s) if stm else self.get_row(s)

    def get_row(self, s):
        """
        This row is based on PySimpleGUI square mapping that is 0 at the
        top and 7 at the bottom.
        In contrast Python-chess square mapping is 0 at the bottom and 7
        at the top. chess.square_rank() is a method from Python-chess that
        returns row given square s.

        :param s: square
        :return: row
        """
        return 7 - chess.square_rank(s)

    def get_col(self, s):
        """ Returns col given square s """
        return chess.square_file(s)

    def redraw_board(self, window):
        """
        Redraw board at start and afte a move.

        :param window:
        :return:
        """
        for i in range(8):
            for j in range(8):
                color = self.sq_dark_color if (i + j) % 2 else self.sq_light_color
                piece_image = images[self.psg_board[i][j]]
                elem = window.find_element(key=(i, j))
                elem.Update(
                    button_color=("white", color),
                    image_filename=piece_image,
                )

    def render_square(self, image, key, location):
        """ Returns an RButton (Read Button) with image image """
        if (location[0] + location[1]) % 2:
            color = self.sq_dark_color  # Dark square
        else:
            color = self.sq_light_color
        return sg.RButton("", image_filename=image, size=(1, 1), border_width=0, button_color=("white", color), pad=(0, 0), key=key)

    def select_promotion_piece(self, stm):
        """
        Allow user to select a piece type to promote to.

        :param stm: side to move
        :return: promoted piece, i.e QUEENW, QUEENB ...
        """
        piece = None
        board_layout, row = [], []

        psg_promote_board = copy.deepcopy(white_init_promote_board) if stm else copy.deepcopy(black_init_promote_board)

        # Loop through board and create buttons with images
        for i in range(1):
            for j in range(4):
                piece_image = images[psg_promote_board[i][j]]
                row.append(self.render_square(piece_image, key=(i, j), location=(i, j)))

            board_layout.append(row)

        promo_window = sg.Window(
            "{} {}".format(APP_NAME, APP_VERSION),
            board_layout,
            default_button_element_size=(12, 1),
            auto_size_buttons=False,
            icon=ico_path[platform]["pecg"],
        )

        while True:
            button, value = promo_window.Read(timeout=0)
            if button is None:
                break
            if type(button) is tuple:
                move_from = button
                fr_row, fr_col = move_from
                piece = psg_promote_board[fr_row][fr_col]
                logging.info("promote piece: {}".format(piece))
                break

        promo_window.Close()

        return piece

    def update_rook(self, window, move):
        """
        Update rook location for castle move.

        :param window:
        :param move: uci move format
        :return:
        """
        if move == "e1g1":
            fr = chess.H1
            to = chess.F1
            pc = ROOKW
        elif move == "e1c1":
            fr = chess.A1
            to = chess.D1
            pc = ROOKW
        elif move == "e8g8":
            fr = chess.H8
            to = chess.F8
            pc = ROOKB
        elif move == "e8c8":
            fr = chess.A8
            to = chess.D8
            pc = ROOKB

        self.psg_board[self.get_row(fr)][self.get_col(fr)] = BLANK
        self.psg_board[self.get_row(to)][self.get_col(to)] = pc
        self.redraw_board(window)

    def update_ep(self, window, move, stm):
        """
        Update board for e.p move.

        :param window:
        :param move: python-chess format
        :param stm: side to move
        :return:
        """
        to = move.to_square
        if stm:
            capture_sq = to - 8
        else:
            capture_sq = to + 8

        self.psg_board[self.get_row(capture_sq)][self.get_col(capture_sq)] = BLANK
        self.redraw_board(window)

    def get_promo_piece(self, move, stm, human):
        """
        Returns promotion piece.

        :param move: python-chess format
        :param stm: side to move
        :param human: if side to move is human this is True
        :return: promoted piece in python-chess and pythonsimplegui formats
        """
        # If this move is from a user, we will show a window with piece images
        if human:
            psg_promo = self.select_promotion_piece(stm)

            # If user pressed x we set the promo to queen
            if psg_promo is None:
                logging.info("User did not select a promotion piece, " "set this to queen.")
                psg_promo = QUEENW if stm else QUEENB

            pyc_promo = promote_psg_to_pyc[psg_promo]
        # Else if move is from computer
        else:
            pyc_promo = move.promotion  # This is from python-chess
            if stm:
                if pyc_promo == chess.QUEEN:
                    psg_promo = QUEENW
                elif pyc_promo == chess.ROOK:
                    psg_promo = ROOKW
                elif pyc_promo == chess.BISHOP:
                    psg_promo = BISHOPW
                elif pyc_promo == chess.KNIGHT:
                    psg_promo = KNIGHTW
            else:
                if pyc_promo == chess.QUEEN:
                    psg_promo = QUEENB
                elif pyc_promo == chess.ROOK:
                    psg_promo = ROOKB
                elif pyc_promo == chess.BISHOP:
                    psg_promo = BISHOPB
                elif pyc_promo == chess.KNIGHT:
                    psg_promo = KNIGHTB

        return pyc_promo, psg_promo

    def set_depth_limit(self):
        """ Returns max depth based from user setting """
        user_depth = sg.PopupGetText(
            "Current depth is {}\n\nInput depth [{} to {}]".format(self.max_depth, MIN_DEPTH, MAX_DEPTH),
            title=BOX_TITLE,
            icon=ico_path[platform]["pecg"],
        )

        try:
            user_depth = int(user_depth)
        except Exception:
            user_depth = self.max_depth
            logging.exception("Failed to get user depth.")

        self.max_depth = min(MAX_DEPTH, max(MIN_DEPTH, user_depth))

    def play_game(self, window, board):
        """
        User can play a game against and engine.

        :param window:
        :param engine_id_name:
        :param board: current board position
        :return:
        """
        window.find_element("_movelist_").Update(disabled=False)
        window.find_element("_movelist_").Update("", disabled=True)

        is_human_stm = True if self.is_user_white else False

        move_state = 0
        move_from, move_to = None, None
        is_new_game, is_exit_game, is_exit_app = False, False, False
        rqst = EmptyRq.Request()
        self._reset_game.call(rqst)

        # Do not play immediately when stm is computer
        is_engine_ready = True if is_human_stm else False

        # For saving game
        move_cnt = 0

        is_user_resigns = False
        is_user_wins = False
        is_user_draws = False
        is_search_stop_for_exit = False
        is_search_stop_for_new_game = False
        is_search_stop_for_neutral = False
        is_search_stop_for_resign = False
        is_search_stop_for_user_wins = False
        is_search_stop_for_user_draws = False

        # Game loop
        while (not board.is_game_over(claim_draw=True)) and (rclpy.ok()):

            moved_piece = None

            # Mode: Play, Hide book 1
            window.Element("polyglot_book1_k").Update("")
            # Mode: Play, Stm: computer (first move), Allow user to change settings.
            # User can start the engine by Engine->Go.
            if is_human_stm:
                if self._robot_moving:
                    self.get_logger().info("Waiting for robot to finish movement")
                    time.sleep(1.0)
                    continue
                move_state = 0

                while True:
                    button, value = window.Read(timeout=100)

                    # Update elapse box in m:s format
                    k = "w_elapse_k"
                    if not self.is_user_white:
                        k = "b_elapse_k"

                    if not is_human_stm:
                        break

                    if button is None:
                        logging.info("Quit app X is pressed.")
                        is_exit_app = True
                        break

                    if is_search_stop_for_exit:
                        is_exit_app = True
                        break

                    # Mode: Play, Stm: User
                    if button == "New::new_game_k" or is_search_stop_for_new_game:
                        is_new_game = True
                        self.clear_elements(window)
                        break

                    # Mode: Play, stm: User
                    if button == "Resign::resign_game_k" or is_search_stop_for_resign:
                        logging.info("User resigns")

                        # Verify resign
                        reply = sg.Popup(
                            "Do you really want to resign?", button_type=sg.POPUP_BUTTONS_YES_NO, title=BOX_TITLE, icon=ico_path[platform]["pecg"]
                        )
                        if reply == "Yes":
                            is_user_resigns = True
                            break
                        else:
                            if is_search_stop_for_resign:
                                is_search_stop_for_resign = False
                            continue

                    # Mode: Play, stm: User
                    if button == "User Wins::user_wins_k" or is_search_stop_for_user_wins:
                        logging.info("User wins by adjudication")
                        is_user_wins = True
                        break

                    # Mode: Play, stm: User
                    if button == "User Draws::user_draws_k" or is_search_stop_for_user_draws:
                        logging.info("User draws by adjudication")
                        is_user_draws = True
                        break

                    # Mode: Play, Stm: User
                    if button == "Neutral" or is_search_stop_for_neutral:
                        is_exit_game = True
                        self.clear_elements(window)
                        break

                    # Mode: Play, stm: User
                    if button == "About":
                        sg.PopupScrolled(
                            HELP_MSG,
                            title=BOX_TITLE,
                        )
                        break

                    # Mode: Play, stm: User
                    if button == "Go":
                        if is_human_stm:
                            is_human_stm = False
                        else:
                            is_human_stm = True
                        is_engine_ready = True
                        window.find_element("_gamestatus_").Update("Mode     Play, Engine is thinking ...")
                        break

                    # Mode: Play, stm: User, user starts moving
                    if type(button) is tuple:
                        # If fr_sq button is pressed
                        if move_state == 0:
                            move_from = button
                            fr_row, fr_col = move_from
                            piece = self.psg_board[fr_row][fr_col]  # get the move-from piece

                            # Change the color of the "fr" board square
                            self.change_square_color(window, fr_row, fr_col)

                            move_state = 1
                            moved_piece = board.piece_type_at(chess.square(fr_col, 7 - fr_row))  # Pawn=1

                        # Else if to_sq button is pressed
                        elif move_state == 1:
                            is_promote = False
                            move_to = button
                            to_row, to_col = move_to
                            button_square = window.find_element(key=(fr_row, fr_col))

                            # If move is cancelled, pressing same button twice
                            if move_to == move_from:
                                # Restore the color of the pressed board square
                                color = self.sq_dark_color if (to_row + to_col) % 2 else self.sq_light_color

                                # Restore the color of the fr square
                                button_square.Update(button_color=("white", color))
                                move_state = 0
                                continue

                            # Create a move in python-chess format based from user input
                            user_move = None
                            # Get the fr_sq and to_sq of the move from user, based from this info
                            # we will create a move based from python-chess format.
                            # Note chess.square() and chess.Move() are from python-chess module
                            fr_row, fr_col = move_from
                            fr_sq = chess.square(fr_col, 7 - fr_row)
                            to_sq = chess.square(to_col, 7 - to_row)
                            type_ = "no_capture"
                            # If user move is a promote
                            if self.relative_row(to_sq, board.turn) == RANK_8 and moved_piece == chess.PAWN:
                                is_promote = True
                                pyc_promo, psg_promo = self.get_promo_piece(user_move, board.turn, True)
                                user_move = chess.Move(fr_sq, to_sq, promotion=pyc_promo)

                            else:
                                user_move = chess.Move(fr_sq, to_sq)
                            # Check if user move is legal
                            if user_move in board.legal_moves:
                                # Update rook location if this is a castle move
                                if board.is_castling(user_move):
                                    type_ = "roque"
                                    self.update_rook(window, str(user_move))

                                # Update board if e.p capture
                                elif board.is_en_passant(user_move):
                                    type_ = "en_passant"
                                    self.update_ep(window, user_move, board.turn)

                                # Empty the board from_square, applied to any types of move
                                self.psg_board[move_from[0]][move_from[1]] = BLANK

                                # Update board to_square if move is a promotion
                                if is_promote:
                                    self.psg_board[to_row][to_col] = psg_promo
                                # Update the to_square if not a promote move
                                else:
                                    # Place piece in the move to_square
                                    self.psg_board[to_row][to_col] = piece

                                self.redraw_board(window)
                                if board.is_capture(user_move):
                                    type_ = "capture"
                                board.push(user_move)
                                move_cnt += 1
                                is_engine_ready = True

                                # Update clock, reset elapse to zero

                                # Update game, move from human
                                user_comment = value["comment_k"]
                                self.update_game(user_move, type_)

                                window.find_element("_movelist_").Update(disabled=False)
                                window.find_element("_movelist_").Update("")
                                window.find_element("_movelist_").Update(self.game.variations[0], append=True, disabled=True)

                                # Clear comment and engine search box
                                window.find_element("comment_k").Update("")
                                window.Element("search_info_all_k").Update("")

                                # Change the color of the "fr" and "to" board squares
                                self.change_square_color(window, fr_row, fr_col)
                                self.change_square_color(window, to_row, to_col)

                                is_human_stm = not is_human_stm

                                window.Element("advise_info_k").Update("")

                            # Else if move is illegal
                            else:
                                move_state = 0
                                color = self.sq_dark_color if (move_from[0] + move_from[1]) % 2 else self.sq_light_color

                                # Restore the color of the fr square
                                button_square.Update(button_color=("white", color))
                                continue

                if is_new_game or is_exit_game or is_exit_app or is_user_resigns or is_user_wins or is_user_draws:
                    break

            # Else if side to move is not human
            elif not is_human_stm and is_engine_ready:
                if self._robot_moving:
                    self.get_logger().info("Waiting for robot to finish movement")
                    time.sleep(1.0)
                    continue
                is_promote = False
                self._get_next_move.wait_for_service()
                answer = self._get_next_move.call(GetNextMove_Request())
                move_cnt += 1

                user_move = answer.move.move
                is_human_stm = True
                is_engine_ready = False
                is_promote = False
                fr_sq = chess.parse_square(answer.move.move[:2])
                fr_row = 7 - fr_sq // 8
                fr_col = fr_sq % 8

                to_sq = chess.parse_square(answer.move.move[2:])
                to_row = 7 - to_sq // 8
                to_col = to_sq % 8
                move_from = (fr_row, fr_col)
                move_to = (to_row, to_col)
                print(move_from, move_to)
                piece = self.psg_board[fr_row][fr_col]
                user_move = chess.Move(fr_sq, to_sq)
                # Check if user move is legal
                if user_move in board.legal_moves:
                    # Update rook location if this is a castle move
                    if board.is_castling(user_move):
                        self.update_rook(window, str(user_move))

                    # Update board if e.p capture
                    elif board.is_en_passant(user_move):
                        self.update_ep(window, user_move, board.turn)

                    # Empty the board from_square, applied to any types of move
                    self.psg_board[move_from[0]][move_from[1]] = BLANK

                    # Update board to_square if move is a promotion
                    if is_promote:
                        self.psg_board[to_row][to_col] = psg_promo
                    # Update the to_square if not a promote move
                    else:
                        # Place piece in the move to_square
                        self.psg_board[to_row][to_col] = piece

                self.redraw_board(window)
                print(user_move)
                board.push(user_move)
                self.update_game(answer.move.move, answer.move.type)
                window.find_element("_movelist_").Update(disabled=False)
                window.find_element("_movelist_").Update("")
                window.find_element("_movelist_").Update(self.game.variations[0], append=True, disabled=True)

        if board.is_game_over(claim_draw=True):
            sg.Popup("Game is over.", title=BOX_TITLE, icon=ico_path[platform]["pecg"])

        if is_exit_app:
            window.Close()
            sys.exit(0)

        self.clear_elements(window)

        return False if is_exit_game else is_new_game

    def save_game(self):
        """ Save game in append mode """
        with open(self.pecg_auto_save_game, mode="a+") as f:
            f.write("{}\n\n".format(self.game))

    def get_engines(self):
        """
        Get engine filenames [a.exe, b.exe, ...]

        :return: list of engine filenames
        """
        engine_list = []
        engine_path = Path("Engines")
        files = os.listdir(engine_path)
        for file in files:
            if (
                not file.endswith(".gz")
                and not file.endswith(".dll")
                and not file.endswith(".DS_Store")
                and not file.endswith(".bin")
                and not file.endswith(".dat")
            ):
                engine_list.append(file)

        return engine_list

    def create_board(self, is_user_white=True):
        """
        Returns board layout based on color of user. If user is white,
        the white pieces will be at the bottom, otherwise at the top.

        :param is_user_white: user has handling the white pieces
        :return: board layout
        """
        file_char_name = "abcdefgh"
        self.psg_board = copy.deepcopy(initial_board)

        board_layout = []

        if is_user_white:
            # Save the board with black at the top
            start = 0
            end = 8
            step = 1
        else:
            start = 7
            end = -1
            step = -1
            file_char_name = file_char_name[::-1]

        # Loop through the board and create buttons with images
        for i in range(start, end, step):
            # Row numbers at left of board is blank
            row = []
            for j in range(start, end, step):
                piece_image = images[self.psg_board[i][j]]
                row.append(self.render_square(piece_image, key=(i, j), location=(i, j)))
            board_layout.append(row)

        return board_layout

    def build_main_layout(self, is_user_white=True):
        """
        Creates all elements for the GUI, icluding the board layout.

        :param is_user_white: if user is white, the white pieces are
        oriented such that the white pieces are at the bottom.
        :return: GUI layout
        """
        sg.ChangeLookAndFeel(self.gui_theme)
        sg.SetOptions(margins=(0, 3), border_width=1)

        # Define board
        board_layout = self.create_board(is_user_white)

        board_controls = [
            [sg.Text("Mode     Neutral", size=(36, 1), font=("Consolas", 10), key="_gamestatus_")],
            [
                sg.Text("White", size=(7, 1), font=("Consolas", 10)),
                sg.Text("Human", font=("Consolas", 10), key="_White_", size=(24, 1), relief="sunken"),
                sg.Text("", font=("Consolas", 10), key="w_base_time_k", size=(11, 1), relief="sunken"),
                sg.Text("", font=("Consolas", 10), key="w_elapse_k", size=(7, 1), relief="sunken"),
            ],
            [
                sg.Text("Black", size=(7, 1), font=("Consolas", 10)),
                sg.Text("Computer", font=("Consolas", 10), key="_Black_", size=(24, 1), relief="sunken"),
                sg.Text("", font=("Consolas", 10), key="b_base_time_k", size=(11, 1), relief="sunken"),
                sg.Text("", font=("Consolas", 10), key="b_elapse_k", size=(7, 1), relief="sunken"),
            ],
            [
                sg.Text(
                    "Adviser",
                    size=(7, 1),
                    font=("Consolas", 10),
                    key="adviser_k",
                    right_click_menu=["Right", ["Start::right_adviser_k", "Stop::right_adviser_k"]],
                ),
                sg.Text("", font=("Consolas", 10), key="advise_info_k", relief="sunken", size=(46, 1)),
            ],
            [sg.Text("Move list", size=(16, 1), font=("Consolas", 10))],
            [sg.Multiline("", do_not_clear=True, autoscroll=True, size=(52, 8), font=("Consolas", 10), key="_movelist_", disabled=True)],
            [sg.Text("Comment", size=(7, 1), font=("Consolas", 10))],
            [sg.Multiline("", do_not_clear=True, autoscroll=True, size=(52, 3), font=("Consolas", 10), key="comment_k")],
            [
                sg.Text(
                    "BOOK 1, Comp games",
                    size=(26, 1),
                    font=("Consolas", 10),
                    right_click_menu=["Right", ["Show::right_book1_k", "Hide::right_book1_k"]],
                ),
                sg.Text("BOOK 2, Human games", font=("Consolas", 10), right_click_menu=["Right", ["Show::right_book2_k", "Hide::right_book2_k"]]),
            ],
            [
                sg.Multiline("", do_not_clear=True, autoscroll=False, size=(23, 4), font=("Consolas", 10), key="polyglot_book1_k", disabled=True),
                sg.Multiline("", do_not_clear=True, autoscroll=False, size=(25, 4), font=("Consolas", 10), key="polyglot_book2_k", disabled=True),
            ],
            [
                sg.Text(
                    "Opponent Search Info",
                    font=("Consolas", 10),
                    size=(30, 1),
                    right_click_menu=["Right", ["Show::right_search_info_k", "Hide::right_search_info_k"]],
                )
            ],
            [sg.Text("", key="search_info_all_k", size=(55, 1), font=("Consolas", 10), relief="sunken")],
        ]

        board_tab = [[sg.Column(board_layout)]]

        self.menu_elem = sg.Menu(menu_def_neutral, tearoff=False)

        # White board layout, mode: Neutral
        layout = [[self.menu_elem], [sg.Column(board_tab), sg.Column(board_controls)]]

        return layout

    def set_default_adviser_engine(self):
        try:
            self.adviser_id_name = self.engine_id_name_list[1] if len(self.engine_id_name_list) >= 2 else self.engine_id_name_list[0]
            self.adviser_file, self.adviser_path_and_file = self.get_engine_file(self.adviser_id_name)
        except IndexError as e:
            logging.warning(e)
        except Exception:
            logging.exception("Error in getting adviser engine!")

    def get_default_engine_opponent(self):
        engine_id_name = None
        try:
            engine_id_name = self.opp_id_name = self.engine_id_name_list[0]
            self.opp_file, self.opp_path_and_file = self.get_engine_file(engine_id_name)
        except IndexError as e:
            logging.warning(e)
        except Exception:
            logging.exception("Error in getting opponent engine!")

        return engine_id_name

    def main_loop(self):
        """
        Build GUI, read user and engine config files and take user inputs.

        :return:
        """
        engine_id_name = None
        layout = self.build_main_layout(True)

        # Use white layout as default window
        window = sg.Window(
            "{} {}".format(APP_NAME, APP_VERSION),
            layout,
            default_button_element_size=(12, 1),
            auto_size_buttons=False,
            icon=ico_path[platform]["pecg"],
        )

        self.init_game()

        # Initialize White and black boxes
        while True:
            button, value = window.Read(timeout=50)
            break

        # Mode: Neutral, main loop starts here
        while rclpy.ok():
            button, value = window.Read(timeout=50)

            # Mode: Neutral
            if button is None:
                logging.info("Quit app from main loop, X is pressed.")
                break

            # Mode: Neutral, Delete player
            # Mode: Neutral
            if button == "Set Depth":
                self.set_depth_limit()
                continue

            # Mode: Neutral, Change theme
            if button in GUI_THEME:
                self.gui_theme = button
                window = self.create_new_window(window)
                continue

            # Mode: Neutral, Change board to gray
            if button == "Gray::board_color_k":
                self.sq_light_color = "#D8D8D8"
                self.sq_dark_color = "#808080"
                self.move_sq_light_color = "#e0e0ad"
                self.move_sq_dark_color = "#999966"
                self.redraw_board(window)
                window = self.create_new_window(window)
                continue

            # Mode: Neutral, Change board to green
            if button == "Green::board_color_k":
                self.sq_light_color = "#daf1e3"
                self.sq_dark_color = "#3a7859"
                self.move_sq_light_color = "#bae58f"
                self.move_sq_dark_color = "#6fbc55"
                self.redraw_board(window)
                window = self.create_new_window(window)
                continue

            # Mode: Neutral, Change board to blue
            if button == "Blue::board_color_k":
                self.sq_light_color = "#b9d6e8"
                self.sq_dark_color = "#4790c0"
                self.move_sq_light_color = "#d2e4ba"
                self.move_sq_dark_color = "#91bc9c"
                self.redraw_board(window)
                window = self.create_new_window(window)
                continue

            # Mode: Neutral, Change board to brown, default
            if button == "Brown::board_color_k":
                self.sq_light_color = "#F0D9B5"
                self.sq_dark_color = "#B58863"
                self.move_sq_light_color = "#E8E18E"
                self.move_sq_dark_color = "#B8AF4E"
                self.redraw_board(window)
                window = self.create_new_window(window)
                continue

            # Mode: Neutral
            if button == "Flip":
                window.find_element("_gamestatus_").Update("Mode     Neutral")
                self.clear_elements(window)
                window = self.create_new_window(window, True)
                continue

            # Mode: Neutral
            if button == "About":
                sg.PopupScrolled(HELP_MSG, title="Help/About")
                continue

            # Mode: Neutral
            if button == "Play":

                # Change menu from Neutral to Play
                self.menu_elem.Update(menu_def_play)
                self.psg_board = copy.deepcopy(initial_board)
                board = chess.Board()

                while rclpy.ok():
                    button, value = window.Read(timeout=100)

                    window.find_element("_gamestatus_").Update("Mode     Play")
                    window.find_element("_movelist_").Update(disabled=False)
                    window.find_element("_movelist_").Update("", disabled=True)

                    start_new_game = self.play_game(window, board)
                    window.find_element("_gamestatus_").Update("Mode     Neutral")

                    self.psg_board = copy.deepcopy(initial_board)
                    self.redraw_board(window)
                    board = chess.Board()
                    self.set_new_game()

                    if not start_new_game:
                        break

                # Restore Neutral menu
                self.menu_elem.Update(menu_def_neutral)
                self.psg_board = copy.deepcopy(initial_board)
                board = chess.Board()
                self.set_new_game()
                continue

        window.Close()


def main():
    rclpy.init()

    theme = "Reddit"

    pecg = EasyChessGui(theme)
    thread = threading.Thread(target=rclpy.spin, args=(pecg,), daemon=True)
    thread.start()

    pecg.main_loop()


if __name__ == "__main__":
    main()
