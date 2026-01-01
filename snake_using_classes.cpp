#include <ncurses.h>
#include <iostream>
#include <vector>
#include <cstdlib>

using namespace std;

/* ---------------- Snake ---------------- */
class Snake {
public:
    int length;
    int head_x, head_y;
    int x_updater, y_updater;
    vector<vector<int>> positions;

    Snake() {
        reset();
    }

    void reset() {
        length = 5;
        head_x = 1;
        head_y = 1;
        x_updater = 1;
        y_updater = 0;
        positions = {{1,1},{1,2},{1,3},{1,4},{1,5}};
    }

    void move() {
        head_x += x_updater;
        head_y += y_updater;

        for (int i = length - 1; i > 0; i--) {
            positions[i] = positions[i - 1];
        }
        positions[0] = {head_x, head_y};
    }

    void draw(WINDOW* win) {
        for (int i = 0; i < length; i++) {
            mvwprintw(win, positions[i][1], positions[i][0], "O");
        }
    }

    bool selfCollision() {
        for (int i = 1; i < length; i++) {
            if (positions[0] == positions[i])
                return true;
        }
        return false;
    }
};

class Game {
public:
    WINDOW* win;
    Snake snake;
    vector<int> apple;
    bool caught;
    bool gameOver;
    int score;

    Game() {
        caught = false;
        gameOver = false;
        score = 0;
        apple = {5, 7};

        win = newwin(20, 40, 5, 10);
        box(win, 0, 0);
    }

    void reset() {
        snake.reset();
        apple = {5, 7};
        caught = false;
        score = 0;
        gameOver = false;
    }

    void run() {
        while (true) {

            if (gameOver) {
                werase(win);
                box(win, 0, 0);
                mvwprintw(win, 5, 5, "GAME OVER");
                mvwprintw(win, 7, 5, "Score: %d", score);
                mvwprintw(win, 9, 5, "Press r to restart");
                mvwprintw(win, 10, 5, "Press q to quit");
                wrefresh(win);

                int ch;
                while (true) {
                    ch = getch();
                    if (ch == 'q') {
                        endwin();
                        return;
                    }
                    if (ch == 'r') {
                        reset();
                        break;
                    }
                }
            }

            werase(win);
            box(win, 0, 0);
            mvprintw(2, 10, "Score: %d", score);

            int ch = getch();

            if (ch == KEY_UP && snake.y_updater != 1) {
                snake.y_updater = -1;
                snake.x_updater = 0;
            } else if (ch == KEY_DOWN && snake.y_updater != -1) {
                snake.y_updater = 1;
                snake.x_updater = 0;
            } else if (ch == KEY_LEFT && snake.x_updater != 1) {
                snake.x_updater = -1;
                snake.y_updater = 0;
            } else if (ch == KEY_RIGHT && snake.x_updater != -1) {
                snake.x_updater = 1;
                snake.y_updater = 0;
            }

            if (caught) {
                apple = {rand() % 18 + 1, rand() % 38 + 1};
                score++;
                caught = false;
            }

            mvwprintw(win, apple[0], apple[1], "A");

            if (snake.head_y == apple[0] && snake.head_x == apple[1]) {
                caught = true;
                snake.positions.push_back(snake.positions.back());
                snake.length++;
            }

            snake.move();

            if (snake.head_x <= 0 || snake.head_x >= 39 || snake.head_y <= 0 || snake.head_y >= 19 || snake.selfCollision()) {
                gameOver = true;
            }

            snake.draw(win);
            wrefresh(win);
            napms(50);
        }
    }
};

int main() {
    initscr();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    timeout(50);

    Game game;
    game.run();

    endwin();
    return 0;
}
