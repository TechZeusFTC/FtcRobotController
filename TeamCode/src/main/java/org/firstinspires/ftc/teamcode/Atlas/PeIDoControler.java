package org.firstinspires.ftc.teamcode.Atlas;

public class PeIDoControler {

    double kP, kI, kD;
    private double cumErro = 0;
    private double erro = 0;
    private double prevErro = 0;

    public PeIDoControler(double kPnovo, double kInovo, double kDnovo) {
        kP = kPnovo;
        kI = kInovo;
        kD = kDnovo;
    }

    public double P(double erro) {
        return kP * erro;
    }

    public double I(double erro) {
        if (!(Math.abs(erro) < 10)) {
            cumErro += erro;
        }
        return cumErro*kI;
    }

    public double D(double erro, double erroAnterior){
        return (erro-erroAnterior) * kD;
    }

    public double CalculatePID(double alvo, double posicao){
        erro = alvo-posicao;
            double power = P(erro)+I(erro)+D(erro,prevErro);
    prevErro = erro;
    return power;
    }
    public void updateConstants(double kPnovo, double kInovo, double kDnovo) {
        kP = kPnovo;
        kI = kInovo;
        kD = kDnovo;
    }
    public  double geterro(){
        return erro;
    }
}