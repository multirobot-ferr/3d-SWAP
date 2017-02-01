#include <qt_gcs/LedIndicator.h>
#include <QPainter>

LedIndicator::LedIndicator(QWidget *parent) :
    QWidget(parent)
{
    setFixedSize(28, 28);
    lit = false;
    ledOnColor=Qt::green;
    ledOffColor=Qt::red;
    ledOnPattern = Qt::SolidPattern;
    ledOffPattern = Qt::SolidPattern;
    ledSize=20;
}

void LedIndicator::paintEvent(QPaintEvent *) {
  QPainter p(this);
  lit ?  p.setBrush(QBrush(ledOnColor, ledOnPattern)) : p.setBrush(QBrush(ledOffColor, ledOffPattern));
  p.drawEllipse(0,0,ledSize,ledSize);
}

void LedIndicator::switchLedIndicator() {
  lit = ! lit;
  update();
}
void LedIndicator::setState(bool state)
{
    lit = state;
    update();
}
void LedIndicator::toggle()
{
    lit = ! lit;
    update();
}

void LedIndicator::setOnColor(QColor onColor)
{
    ledOnColor=onColor;
    update();
}
void LedIndicator::setOffColor(QColor offColor)
{
    ledOffColor=offColor;
    update();
}
void LedIndicator::setOnPattern(Qt::BrushStyle onPattern)
{
    ledOnPattern=onPattern;
    update();
}
void LedIndicator::setOffPattern(Qt::BrushStyle offPattern)
{
    ledOffPattern=offPattern;
    update();
}
void LedIndicator::setLedSize(int size)
{
    ledSize=size;
    setFixedSize(size+10, size+10);
    update();
}
