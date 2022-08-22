# Build: sudo docker build -t clover-docs:latest .

# Run: docker run -it --rm -v ~/clover:/clover clover-docs:latest

# Save image: sudo docker save clover-docs:latest | gzip > clover-docs.tar.gz

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y curl git

RUN curl https://raw.githubusercontent.com/creationix/nvm/master/install.sh | bash
RUN . $HOME/.nvm/nvm.sh && nvm install 10.15 && nvm use 10.15 && npm --version

RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections \
  && apt-get update \
  && apt-get install -y calibre msttcorefonts ghostscript

RUN . $HOME/.nvm/nvm.sh && curl https://raw.githubusercontent.com/CopterExpress/clover/master/builder/assets/install_gitbook.sh | bash

# RUN . $HOME/.nvm/nvm.sh && npm install markdownlint-cli@0.28.1 -g && PUPPETEER_SKIP_DOWNLOAD=1 npm install svgexport -g
RUN . $HOME/.nvm/nvm.sh && npm install markdownlint-cli@0.28.1 -g && npm install svgexport -g

RUN . $HOME/.nvm/nvm.sh && node -v && gitbook -V && markdownlint -V

RUN curl https://raw.githubusercontent.com/CopterExpress/clover/master/book.json > book.json

RUN . $HOME/.nvm/nvm.sh && gitbook install

CMD . $HOME/.nvm/nvm.sh && cp -r node_modules /clover && cd /clover && gitbook build \
  && gitbook pdf ./ _book/clover.pdf && \
  gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/default -dNOPAUSE -dQUIET -dBATCH -dDetectDuplicateImages -dCompressFonts=true -r150 -sOutputFile=_book/clover_ru_compressed.pdf _book/clover_ru.pdf && \
  gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/default -dNOPAUSE -dQUIET -dBATCH -dDetectDuplicateImages -dCompressFonts=true -r150 -sOutputFile=_book/clover_en_compressed.pdf _book/clover_en.pdf && \
  rm _book/clover_ru.pdf && mv _book/clover_ru_compressed.pdf _book/clover_ru.pdf && \
  rm _book/clover_en.pdf && mv _book/clover_en_compressed.pdf _book/clover_en.pdf
