#pragma once

#include<bits/stdc++.h>

#define NOBOARD 0
#define EMPTY 1
#define O 2
#define X 3

class ChessPlayer{
    public:
    ChessPlayer();
    ~ChessPlayer(){};

    void SetState(int board[3][3]);

    void PlayChess();

    int AlphaBeta(int &value,int deep,bool MAX);
    int Value();
    bool Win();

    int GetI();
    int GetJ();

    private:
    enum PlayerState{NOT_INITIALIZED=0, STATE_SET, STATE_SOLVED};
    PlayerState _playerstate;
    int _board[3][3];
    int _decisionI;
    int _decisionJ;

    int AlphaBetaChess[3][3];
    int AlphaBetacount;
};