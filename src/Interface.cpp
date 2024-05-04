#include "Interface.h"
#include "input.h"
#include "ansi.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace std;

bool Interface::init(){
    if (!categoryMenu())
        return false;
    clearScreen();
    return true;
}

void Interface::clearScreen() {
#ifdef __unix__
    system("clear");
#else
    // assume windows
    system("cls");
#endif
}

void Interface::printTop() {
    std::string s;
    for (int _ = 0; _ < 78; _++){
        s += "─";
    }
    std::cout << "┌" << s << "┐" << '\n';
    std::string title = "Travelling Salesperson Problem (G15_02)";
    std::cout << "│" << std::string(20, ' ') << CYAN << BOLD << title << RESET << std::string(19, ' ') << "│" << '\n';
}

void Interface::printBottom() {
    std::string s;
    for (int _ = 0; _ < 78; _++){
        s += "─";
    }
    std::cout << "└" << s << "┘" << '\n';
}

void Interface::waitInput() {
    initCapture();
    std::cout << HIDE_CURSOR;
    cout << '\n' << std::string(25, ' ') << FAINT << "< Press " << RESET << BOLD << "ENTER" << RESET << FAINT << " to continue >\n" << RESET;

    Press press;
    do {
        press = getNextPress();
    } while  (press != RET);
    endCapture();
}

void Interface::printMenuOptions(const std::vector<std::string> &options, int choice) {
    std::cout << "│" << std::string(4, ' ') << std::setw(74) << std::left << options[options.size()-1] << "│" << '\n';

    for (int idx = 1; idx < options.size() - 2; idx++){
        int space = 73;
        if (idx >= 10){
            space--;
        }
        if (choice == idx){
            std::cout << "│" << BOLD << GREEN << " [" << idx << "] " << RESET << BOLD << std::setw(space) << std::left << options[idx] << RESET << "│" << '\n';
        }
        else {
            std::cout << "│" << GREEN << " [" << idx << "] " << RESET << FAINT << std::setw(space) << std::left << options[idx] << RESET << "│" << '\n';
        }
    }

    int space = 73;
    if (options.size() - 2 >= 10){
        space--;
    }

    if (choice == options.size()-2){
        std::cout << "│" << BOLD << YELLOW << " [" << options.size()-2 << "] " << RESET << BOLD << std::setw(space) << std::left << options[options.size()-2] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << YELLOW << " [" << options.size()-2 << "] " << RESET << FAINT << std::setw(space) << std::left << options[options.size()-2] << RESET << "│" << '\n';
    }

    if (choice == 0){
        std::cout << "│" << BOLD << RED << " [0] " << RESET << BOLD << std::setw(73) << std::left << options[0] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << RED << " [0] " << RESET << FAINT << std::setw(73) << std::left << options[0] << RESET << "│" << '\n';
    }
}

void Interface::printMenuOptionsNoBottom(const std::vector<std::string> &options, int choice) {
    std::cout << "│" << std::string(4, ' ') << std::setw(74) << std::left << options[options.size()-1] << "│" << '\n';

    for (int idx = 1; idx < options.size() - 1; idx++){
        int w = 73;
        if (idx >= 10) {
            w--;
        }
        if (choice == idx){
            std::cout << "│" << BOLD << GREEN << " [" << idx << "] " << RESET << BOLD << std::setw(w) << std::left << options[idx] << RESET << "│" << '\n';
        }
        else {
            std::cout << "│" << GREEN << " [" << idx << "] " << RESET << FAINT << std::setw(w) << std::left << options[idx] << RESET << "│" << '\n';
        }
    }
    if (choice == 0){
        std::cout << "│" << BOLD << RED << " [0] " << RESET << BOLD << std::setw(73) << std::left << options[0] << RESET "│" << '\n';
    }
    else {
        std::cout << "│" << RED << " [0] " << RESET << FAINT << std::setw(73) << std::left << options[0] << RESET << "│" << '\n';
    }
}

void Interface::mainMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Quit",
             "Backtracking Algorithm",
             "Held-Karp Algorithm",
             "Double Minimum Spanning Tree Heuristic",
             "Nearest Neighbour Heuristic",
             "Christofides* Heuristic",
             "Real World Heuristic",
             "Choose your operation:"
            };


    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printMenuOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);
    endCapture();

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    double result;
    switch (choice) {
        case 1: {
            start = chrono::high_resolution_clock::now();
            result = graph->backtrackingTSP();
            end = chrono::high_resolution_clock::now();
            backtrackResult = result;
            break;
        }
        case 2: {
            start = chrono::high_resolution_clock::now();
            result = graph->heldKarpTSP();
            end = chrono::high_resolution_clock::now();
            heldKarpResult = result;
            break;
        }
        case 3: {
            start = chrono::high_resolution_clock::now();
            result = graph->doubleMSTTSP();
            end = chrono::high_resolution_clock::now();
            doubleMSTResult = result;
            break;
        }
        case 4: {
            start = chrono::high_resolution_clock::now();
            result = graph->nearestNeighbourTSP();
            end = chrono::high_resolution_clock::now();
            nearestNeighborResult = result;
            break;
        }
        case 5: {
            start = chrono::high_resolution_clock::now();
            result = graph->christofidesTSP();
            end = chrono::high_resolution_clock::now();
            christofidesResult = result;
            break;
        }
        case 6: {
            start = chrono::high_resolution_clock::now();
            result = graph->realWorldTSP();
            end = chrono::high_resolution_clock::now();
            realWorldResult = result;
            break;
        }
        case 0:
            exitMenu();
            break;
    }
}

bool Interface::categoryMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Quit",
             "Extra Fully Connected Graphs",
             "Real World Graphs",
             "Toy Graphs",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printMenuOptionsNoBottom(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: {
            return extraFullyConnectedMenu();
        }
        case 2: {
            return realWorldMenu();
        }
        case 3: {
            return toyMenu();
        }
        default:
            exitMenu();
    }
}

bool Interface::extraFullyConnectedMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Back",
             "Extra Fully Connected: 25 edges",
             "Extra Fully Connected: 50 edges",
             "Extra Fully Connected: 75 edges",
             "Extra Fully Connected: 100 edges",
             "Extra Fully Connected: 200 edges",
             "Extra Fully Connected: 300 edges",
             "Extra Fully Connected: 400 edges",
             "Extra Fully Connected: 500 edges",
             "Extra Fully Connected: 600 edges",
             "Extra Fully Connected: 700 edges",
             "Extra Fully Connected: 800 edges",
             "Extra Fully Connected: 900 edges",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printMenuOptionsNoBottom(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: return parse(graph, "graphs/Extra Fully Connected/edges_25.csv");;
        case 2: return parse(graph, "graphs/Extra Fully Connected/edges_50.csv");
        case 3: return parse(graph, "graphs/Extra Fully Connected/edges_75.csv");
        case 4: return parse(graph, "graphs/Extra Fully Connected/edges_100.csv");
        case 5: return parse(graph, "graphs/Extra Fully Connected/edges_200.csv");
        case 6: return parse(graph, "graphs/Extra Fully Connected/edges_300.csv");
        case 7: return parse(graph, "graphs/Extra Fully Connected/edges_400.csv");
        case 8: return parse(graph, "graphs/Extra Fully Connected/edges_500.csv");
        case 9: return parse(graph, "graphs/Extra Fully Connected/edges_600.csv");
        case 10: return parse(graph, "graphs/Extra Fully Connected/edges_700.csv");
        case 11: return parse(graph, "graphs/Extra Fully Connected/edges_800.csv");
        case 12: return parse(graph, "graphs/Extra Fully Connected/edges_900.csv");
        default:
            return categoryMenu();
    }
}

bool Interface::realWorldMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Back",
             "Real World Graphs: 1",
             "Real World Graphs: 2",
             "Real World Graphs: 3",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printMenuOptionsNoBottom(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: return parse(graph, "graphs/Real World/graph1/edges.csv");
        case 2: return parse(graph, "graphs/Real World/graph2/edges.csv");
        case 3: return parse(graph, "graphs/Real World/graph3/edges.csv");
        default:
            return categoryMenu();
    }
}

bool Interface::toyMenu() {
    initCapture();
    std::cout << HIDE_CURSOR;
    std::vector<std::string> options =
            {"Back",
             "Toy Graphs: Shipping",
             "Toy Graphs: Stadiums",
             "Toy Graphs: Tourism",
             "Choose the Dataset"
            };

    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printMenuOptionsNoBottom(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: return parse(graph, "graphs/Toy/shipping.csv");
        case 2: return parse(graph, "graphs/Toy/stadiums.csv");
        case 3: return parse(graph, "graphs/Toy/tourism.csv");
        default:
            return categoryMenu();
    }
}

void Interface::exitMenu() {
    std::cout << "Closing the app...\n";
    exit(0);
}