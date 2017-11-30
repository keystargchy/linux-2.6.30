
    DEST=/home/tftp/uImage_sbc6845

    export PATH=~/bin/arm/arm-2007q1/bin:$PATH

    #make sbc6845_defconfig

    make uImage -j 2
    #make modules

    echo "uImage -> $DEST"
    cp arch/arm/boot/uImage $DEST
