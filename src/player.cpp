#include "dobot/player.h"

ChessPlayer::ChessPlayer(){
    _playerstate = ChessPlayer::NOT_INITIALIZED;
}

ChessPlayer::~ChessPlayer(){

}


void ChessPlayer::SetState(int msg){
    _msg = _msg;
    _board = 0.1;

    _playerstate = ChessPlayer::STATE_SET;
}

void ChessPlayer::PlayChess(){
    if(_playerstate == ChessPlayer::NOT_INITIALIZED){
        throw std::runtime_error("state not initialized");
        return;
    }

    _decision = _board*20;

    _playerstate = ChessPlayer::STATE_SOLVED;
}

int ChessPlayer::Getlocation(){
    if(_playerstate != ChessPlayer::STATE_SOLVED){
        throw std::runtime_error("state not solved");
        return 0;
    }

    return _decision;
}