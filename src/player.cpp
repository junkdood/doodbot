#include "doodbot/player.h"

ChessPlayer::ChessPlayer(){
    _playerstate = ChessPlayer::NOT_INITIALIZED;
}


void ChessPlayer::SetState(int board[3][3]){
    for(int i = 0; i<3;i++){
        for(int j = 0; j<3;j++){
            _board[i][j]= board[i][j];
        }
    }
    _playerstate = ChessPlayer::STATE_SET;
}

void ChessPlayer::PlayChess(){
    //默认有空位置，棋盘已识别
    if(_playerstate == ChessPlayer::NOT_INITIALIZED){
        throw std::runtime_error("state not initialized");
        return;
    }
    do{
        _decisionI = rand()%3;
        _decisionJ = rand()%3;
    }while(_board[_decisionI][_decisionJ]!=EMPTY);
    
    _playerstate = ChessPlayer::STATE_SOLVED;
}

int ChessPlayer::GetI(){
    if(_playerstate != ChessPlayer::STATE_SOLVED){
        throw std::runtime_error("state not solved");
        return 0;
    }

    return _decisionI;
}
int ChessPlayer::GetJ(){
    if(_playerstate != ChessPlayer::STATE_SOLVED){
        throw std::runtime_error("state not solved");
        return 0;
    }

    return _decisionJ;
}