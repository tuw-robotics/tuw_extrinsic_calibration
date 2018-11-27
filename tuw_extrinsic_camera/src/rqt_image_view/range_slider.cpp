/**
 * @author: Felix Koenig felix.koenig@tuwien.ac.at, felix.koenig@protonmail.com
 */
//Project
#include <rqt_image_view/range_slider.h>

//Qt
#include <QMouseEvent>
#include "qmimedata.h"
#include "qdrag.h"
#include "qwidgetaction.h"
#include "qapplication.h"
#include "qpixmap.h"
#include "qcursor.h"
#include "qguiapplication.h"
#include "qdir.h"
#include <QProxyStyle>

class SliderProxy : public QProxyStyle {
public:
  int pixelMetric( PixelMetric metric, const QStyleOption *option = 0, const QWidget *widget = 0 ) const {
    switch ( metric ) {
      case PM_SliderThickness  :
        return 25;
      case PM_SliderLength     :
        return 25;
      default                  :
        return (QProxyStyle::pixelMetric( metric, option, widget ));
    }
  }
};

RangeSlider::RangeSlider( QWidget *parent )
    : QSlider( parent ) {
  //styling
  setOrientation( Qt::Horizontal );
  setAcceptDrops( true );
  //@ToDo: reintroduce
  // SliderProxy *aSliderProxy = new SliderProxy();
  //
  // //hard coded path to image :/ sorry
  // QString path = QDir::fromNativeSeparators( ImagesPath( "handle.png" ));
  // setStyleSheet( "QSlider::handle { image: url(" + path + "); }" );
  // setStyle( parent->style() );
  
  //setting up the alternate handle
  alt_handle = new RangeSliderHandle( this );
  addAction( new QWidgetAction( alt_handle ));
  alt_handle->move( this->pos().x() + this->width() - alt_handle->width(), this->pos().y());
  
}

RangeSliderHandle::RangeSliderHandle( RangeSlider *_parent )
    : QLabel( _parent ) {
  parent = _parent;
  filter = new SliderEventFilter( parent );
  
  //styling
  setAcceptDrops( true );
  //hard coded path to image :/ sorry
  //@ToDo: reintroduce
  //QPixmap pix = QPixmap( ImagesPath( "handle.png" ));
  //pix = pix.scaled( 25, 25, Qt::IgnoreAspectRatio, Qt::SmoothTransformation );
  //setPixmap( pix );
}

int RangeSlider::alt_value() {
  return alt_handle->value();
}

void RangeSlider::alt_setValue( int value ) {
  alt_handle->setValue( value );
}

void RangeSlider::mouseReleaseEvent( QMouseEvent *mouseEvent ) {
  if ( mouseEvent->button() == Qt::LeftButton ) {
    alt_handle->show();
    alt_handle->handleActivated = false;
  }
  mouseEvent->accept();
}

void RangeSlider::alt_update() {
  QPoint posCursor( QCursor::pos());
  QPoint posParent( mapToParent(mapToGlobal( pos())));
  QPoint point( alt_handle->mapToParent( alt_handle->mapFromGlobal( QCursor::pos())).x(), alt_handle->y());
  int horBuffer = (alt_handle->width());
  bool lessThanMax = mapToParent( point ).x() < pos().x() + width() - horBuffer;
  bool greaterThanMin = mapToParent( point ).x() > pos().x();
  if ( lessThanMax && greaterThanMin )
    alt_handle->move( point );
  emit alt_valueChanged( alt_value());
}

void RangeSliderHandle::mousePressEvent( QMouseEvent *mouseEvent ) {
  qGuiApp->installEventFilter( filter );
  parent->clearFocus();
}

bool SliderEventFilter::eventFilter( QObject *obj, QEvent *event ) {
  switch ( event->type()) {
    case QEvent::MouseButtonRelease:
      qGuiApp->removeEventFilter( this );
      return true;
      break;
    case QEvent::MouseMove:
      grandParent->alt_update();
      return true;
      break;
    default:
      return QObject::eventFilter( obj, event );
  }
  return false;
}

void RangeSliderHandle::setValue( double value ) {
  double width = parent->width(), position = pos().x();
  double range = parent->maximum() - parent->minimum();
  int location = (value - parent->minimum()) / range;
  location = location * width;
  move( y(), location );
}

int RangeSliderHandle::value() {
  double width = parent->width(), position = pos().x();
  double value = position / width;
  double range = parent->maximum() - parent->minimum();
  return parent->minimum() + (value * range);
}

void RangeSlider::Reset() {
  int horBuffer = (alt_handle->width());
  QPoint myPos = mapToGlobal( pos());
  QPoint point( myPos.x() + width() - horBuffer, myPos.y() - alt_handle->height());
  point = alt_handle->mapFromParent( point );
  
  alt_handle->move( point );
  alt_handle->show();
  alt_handle->raise();
  
}
