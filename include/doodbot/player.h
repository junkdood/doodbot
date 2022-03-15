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

    void SetState(int msg);

    void PlayChess();

    int Getlocation();

    private:
    enum PlayerState{NOT_INITIALIZED=0, STATE_SET, STATE_SOLVED};
    PlayerState _playerstate;
    double _board;
    int _msg;
    int _decision;
};