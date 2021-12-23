// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RQT_IMAGE_OVERLAY__RATIO_LAYOUTED_FRAME_HPP_
#define RQT_IMAGE_OVERLAY__RATIO_LAYOUTED_FRAME_HPP_

#include <QFrame>
#include <QImage>
#include <memory>
#include <string>

namespace rqt_image_overlay
{

class RatioLayoutedFrame
  : public QFrame
{
  Q_OBJECT

public:
  explicit RatioLayoutedFrame(QWidget * parent, Qt::WindowFlags flags = 0);
  void setImage(std::unique_ptr<QImage> image);

signals:
  void delayedUpdate();

protected:
  void paintEvent(QPaintEvent * event);

private:
  std::unique_ptr<QImage> qimage;
};

}  // namespace rqt_image_overlay

#endif  // RQT_IMAGE_OVERLAY__RATIO_LAYOUTED_FRAME_HPP_
