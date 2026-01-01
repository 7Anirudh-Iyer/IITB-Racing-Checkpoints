#include <ncurses.h>
#include <iostream>
#include <vector>
#include <cstdlib>

using namespace std;

int main() {
    initscr(); 
    noecho();
    curs_set(0);

    keypad(stdscr, TRUE);
    timeout(50);

    WINDOW* win = newwin(20, 40, 5, 10);
    box(win, 0, 0);

    int snake_length = 5;
    int head_x = 1;
    int head_y = 1;
    int x_updater = 1;
    int y_updater = 0;
    bool caught = false;
    int score = 0;
    bool gameOver = false;

    vector<vector<int>> positions = {{head_x, head_y}, {1, 2}, {1, 3}, {1, 4}, {1, 5}};
    vector<int> apple = {5, 7};

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
                    return 0;
                }
                if (ch == 'r') {
                    snake_length = 5;
                    head_x = 1;
                    head_y = 1;
                    x_updater = 1;
                    y_updater = 0;
                    caught = false;
                    score = 0;
                    positions = {{1,1},{1,2},{1,3},{1,4},{1,5}};
                    apple = {5,7};
                    gameOver = false;
                    break;
                }
            }
        }

        werase(win);
        box(win, 0, 0);

        mvprintw(2, 10, "Score: %d", score);

        int ch = getch();

        if (ch == KEY_UP) {
            if (y_updater != 1) {
                y_updater = -1;
                x_updater = 0;
            } else {
                gameOver = true;
            }
        }
        else if (ch == KEY_DOWN) {
            if (y_updater != -1) {
                y_updater = 1;
                x_updater = 0;
            } else {
                gameOver = true;
            }
        }
        else if (ch == KEY_LEFT) {
            if (x_updater != 1) {
                x_updater = -1;
                y_updater = 0;
            } else {
                gameOver = true;
            }
        }
        else if (ch == KEY_RIGHT) {
            if (x_updater != -1) {
                x_updater = 1;
                y_updater = 0;
            } else {
                gameOver = true;
            }
        }

        if (caught == true) {
            int a = rand() % 18 + 1;
            int b = rand() % 38 + 1;
            apple = {a, b};
            mvwprintw(win, a, b, "A");
            score += 1;
            caught = false;
        } else {
            mvwprintw(win, apple[0], apple[1], "A");
        }

        if (head_y == apple[0] && head_x == apple[1]) {
            caught = true;
            vector<int> new_pos = {positions[snake_length-1][0], positions[snake_length-1][1]};
            snake_length += 1;
            positions.push_back(new_pos);
        }

        head_x += x_updater;
        head_y += y_updater;

        if (head_x <= 0 || head_x >= 39 || head_y <= 0 || head_y >= 19) {
            gameOver = true;
        }

        mvwprintw(win, positions[0][1], positions[0][0], "O");
        for (int i = snake_length-1; i > 0; i--) {
            mvwprintw(win, positions[i][1], positions[i][0], "O");
            positions[i][0] = positions[i-1][0];
            positions[i][1] = positions[i-1][1];
        }
        positions[0] = {head_x, head_y};

        for (int i = 1; i < snake_length; i++) {
            if (positions[0] == positions[i]) {
                gameOver = true;
            }
        }

        wrefresh(win);
        napms(50);
    }

    endwin();
    return 0;
}
