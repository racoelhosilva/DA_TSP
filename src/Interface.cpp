#include "Interface.h"
#include "input.h"
#include "ansi.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>

using namespace std;

bool Interface::init(){
    categoryMenu();
    if (graph == nullptr){
        return false;
    }
    clearScreen();
    return true;
}

void Interface::clearScreen() {
#ifdef __unix__
    system("clear");
#else
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

void Interface::printDatasetsOptions(const vector<std::string> &options, int choice){
    std::cout << "│" << std::string(4, ' ') << std::setw(74) << std::left << options[options.size()-1] << "│" << '\n';

    for (int idx = 1; idx < (int)options.size() - 1; idx++){
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

void Interface::printAlgorithmOptions(const vector<std::string> &options, int choice){
    std::cout << "│" << std::string(4, ' ') << std::setw(74) << std::left << options[options.size()-1] << "│" << '\n';

    for (int idx = 1; idx < (int)options.size() - 2; idx++){
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

    if (choice == (int)options.size()-2){
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
             "Print Graph",
             "Statistics",
             "Choose your operation:"
            };


    int choice = 1;
    Press press;
    do {
        clearScreen();
        printTop();
        printAlgorithmOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);
    endCapture();

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    double result = NAN;
    switch (choice) {
        case 1: {
            start = chrono::high_resolution_clock::now();
            result = graph->backtrackingTsp();
            end = chrono::high_resolution_clock::now();
            backtrackResult = result;
            break;
        }
        case 2: {
            start = chrono::high_resolution_clock::now();
            result = graph->heldKarpTsp();
            end = chrono::high_resolution_clock::now();
            heldKarpResult = result;
            break;
        }
        case 3: {
            start = chrono::high_resolution_clock::now();
            result = graph->doubleMstTsp(0);
            end = chrono::high_resolution_clock::now();
            doubleMSTResult = result;
            break;
        }
        case 4: {
            start = chrono::high_resolution_clock::now();
            result = graph->nearestNeighbourTsp(0);
            end = chrono::high_resolution_clock::now();
            nearestNeighborResult = result;
            break;
        }
        case 5: {
            start = chrono::high_resolution_clock::now();
            result = graph->christofidesTsp(0);
            end = chrono::high_resolution_clock::now();
            christofidesResult = result;
            break;
        }
        case 6: {
            start = chrono::high_resolution_clock::now();
            result = graph->realWorldTsp(0);
            end = chrono::high_resolution_clock::now();
            realWorldResult = result;
            break;
        }
        case 7: {
            for (Vertex *v: graph->getVertexSet()) {
                cout << v->getId() << ": ";
                for (auto e: v->getAdj()) {
                    cout << e->getDest()->getId() << '(' << e->getWeight() << ')' << ' ';
                }
                cout << '\n';
            }

            cout << graph->getNumEdges() << '\n';
            waitInput();
            return;
        }
        case 8: {
            statistics();
            waitInput();
            return;
        }
        case 0:
            exitMenu();
            break;
    }

    std::chrono::duration<double> execution = end - start;
    cout << "Result: " << fixed << setprecision(3) << result << '\n';
    cout << "Execution: " << fixed << setprecision(10) << execution.count() << '\n';
    waitInput();
}

void Interface::categoryMenu() {
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
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: extraFullyConnectedMenu(); break;
        case 2: realWorldMenu(); break;
        case 3: toyMenu(); break;
        default:
            exitMenu();
    }
}

void Interface::extraFullyConnectedMenu() {
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
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_25.csv"); break;
        case 2: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_50.csv"); break;
        case 3: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_75.csv"); break;
        case 4: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_100.csv"); break;
        case 5: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_200.csv"); break;
        case 6: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_300.csv"); break;
        case 7: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_400.csv"); break;
        case 8: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_500.csv"); break;
        case 9: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_600.csv"); break;
        case 10: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_700.csv"); break;
        case 11: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_800.csv"); break;
        case 12: graph = Graph::parseMediumGraph("../graphs/Extra Fully Connected/nodes.csv", "../graphs/Extra Fully Connected/edges_900.csv"); break;
        default:
            categoryMenu();
    }
}

void Interface::realWorldMenu() {
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
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: graph = Graph::parseRealWorldGraph("../graphs/Real World/graph1/nodes.csv", "../graphs/Real World/graph1/edges.csv"); break;
        case 2: graph = Graph::parseRealWorldGraph("../graphs/Real World/graph2/nodes.csv", "../graphs/Real World/graph2/edges.csv"); break;
        case 3: graph = Graph::parseRealWorldGraph("../graphs/Real World/graph3/nodes.csv", "../graphs/Real World/graph3/edges.csv"); break;
        default:
            categoryMenu();
    }
}

void Interface::toyMenu() {
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
        printDatasetsOptions(options, choice);
        printBottom();
        press = getNextPress();
        if (press == UP) {choice -= 1; choice += (options.size()-1);}
        else if (press == DOWN) {choice += 1;}
        choice = choice % (options.size()-1);
    } while (press != RET);

    endCapture();

    switch (choice) {
        case 1: graph = Graph::parseToyGraph("../graphs/Toy/shipping.csv"); break;
        case 2: graph = Graph::parseToyGraph("../graphs/Toy/stadiums.csv"); break;
        case 3: graph = Graph::parseToyGraph("../graphs/Toy/tourism.csv"); break;
        default:
            categoryMenu();
    }
}

void Interface::statistics() {
    cout << "Statistics :)\n";
}

void Interface::exitMenu() {
    std::cout << "Closing the app...\n";
    exit(0);
}